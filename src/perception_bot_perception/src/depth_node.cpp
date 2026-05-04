// depth_node — runs Depth Anything V2 on /camera/image and republishes
// Gazebo's ground-truth depth, so downstream consumers can choose either.
//
// Topics:
//   sub  /camera/image          (RGB)
//   sub  /camera/depth_image    (Gazebo GT depth, used for one-time calibration)
//   pub  /perception/depth_estimated  (DA-V2 output, metric, 32FC1)
//   pub  /perception/depth_gt         (passthrough of /camera/depth_image)
//
// The estimated-depth pipeline only runs when the package was built with
// ONNX Runtime. Without ONNX, depth_node is not built (CMakeLists guard).
#ifdef USE_ONNXRUNTIME

#include "perception_bot_perception/depth_anything.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace perception_bot {

class DepthNode : public rclcpp::Node {
public:
  DepthNode() : rclcpp::Node("depth_node") {
    DepthAnythingConfig cfg;
    cfg.model_path = declare_parameter<std::string>("model_path", "");
    cfg.use_cuda = declare_parameter<bool>("use_cuda", true);
    cfg.input_size = declare_parameter<int>("input_size", 518);
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image");
    depth_gt_topic_ = declare_parameter<std::string>(
        "depth_gt_topic", "/camera/depth_image");
    depth_est_topic_ = declare_parameter<std::string>(
        "depth_estimated_topic", "/perception/depth_estimated");
    depth_passthrough_topic_ = declare_parameter<std::string>(
        "depth_gt_relay_topic", "/perception/depth_gt");

    if (cfg.model_path.empty()) {
      RCLCPP_WARN(get_logger(),
          "model_path is empty — depth_node will only relay /camera/depth_image");
    } else {
      try {
        depth_anything_ = std::make_unique<DepthAnything>(std::move(cfg));
        RCLCPP_INFO(get_logger(), "Loaded Depth Anything V2 model");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to load DA-V2: %s", e.what());
      }
    }

    pub_est_ = create_publisher<sensor_msgs::msg::Image>(depth_est_topic_, 10);
    pub_gt_relay_ = create_publisher<sensor_msgs::msg::Image>(
        depth_passthrough_topic_, 10);

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          on_image(std::move(msg));
        });
    sub_gt_ = create_subscription<sensor_msgs::msg::Image>(
        depth_gt_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          on_gt(std::move(msg));
        });

    RCLCPP_INFO(get_logger(),
        "depth_node ready: %s + %s → %s & %s",
        image_topic_.c_str(), depth_gt_topic_.c_str(),
        depth_est_topic_.c_str(), depth_passthrough_topic_.c_str());
  }

private:
  void on_gt(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    pub_gt_relay_->publish(*msg);
    std::lock_guard<std::mutex> lk(mtx_);
    last_gt_ = std::move(msg);
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!depth_anything_) return;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    if (!depth_anything_->is_calibrated()) {
      sensor_msgs::msg::Image::ConstSharedPtr gt;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        gt = last_gt_;
      }
      if (!gt) {
        RCLCPP_DEBUG(get_logger(),
            "Waiting for first GT depth frame before calibrating DA-V2");
        return;
      }
      try {
        cv::Mat gt_mat = cv_bridge::toCvCopy(gt, "32FC1")->image;
        depth_anything_->calibrate_against_gt(cv_ptr->image, gt_mat);
        RCLCPP_INFO(get_logger(), "DA-V2 metric calibration complete");
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "DA-V2 calibration failed: %s", e.what());
        return;
      }
    }

    cv::Mat metric = depth_anything_->infer(cv_ptr->image);
    auto out = cv_bridge::CvImage(msg->header, "32FC1", metric).toImageMsg();
    pub_est_->publish(*out);
  }

  std::string image_topic_;
  std::string depth_gt_topic_;
  std::string depth_est_topic_;
  std::string depth_passthrough_topic_;
  std::unique_ptr<DepthAnything> depth_anything_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_gt_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_est_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_gt_relay_;
  std::mutex mtx_;
  sensor_msgs::msg::Image::ConstSharedPtr last_gt_;
};

}  // namespace perception_bot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_bot::DepthNode>());
  rclcpp::shutdown();
  return 0;
}

#else  // USE_ONNXRUNTIME

#include <cstdio>
int main() {
  std::fprintf(stderr,
      "depth_node was built without ONNX Runtime support; install the\n"
      "/opt/onnxruntime tarball (scripts/install_onnxruntime.sh) and rebuild.\n");
  return 1;
}

#endif  // USE_ONNXRUNTIME
