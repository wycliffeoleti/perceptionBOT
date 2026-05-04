// detection_node — subscribes /camera/image, runs the configured detector
// backend (HSV by default; YOLOv11n ONNX optionally), and publishes
// /perception/detections (perception_bot_msgs::DetectedObjectArray).
#include "perception_bot_perception/detector.hpp"
#include "perception_bot_perception/hsv_detector.hpp"
#ifdef USE_ONNXRUNTIME
#include "perception_bot_perception/yolo_v11.hpp"
#endif

#include <cv_bridge/cv_bridge.hpp>
#include <perception_bot_msgs/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace perception_bot {

class DetectionNode : public rclcpp::Node {
public:
  DetectionNode() : rclcpp::Node("detection_node") {
    backend_ = declare_parameter<std::string>("backend", "hsv");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/perception/detections");

    declare_parameter<std::string>("model_path", "");
    declare_parameter<bool>("use_cuda", true);
    declare_parameter<double>("conf_threshold", 0.25);
    declare_parameter<double>("iou_threshold", 0.45);
    declare_parameter<int>("min_area_px", 200);

    if (backend_ == "hsv") {
      HsvDetectorConfig cfg;
      cfg.min_area_px = get_parameter("min_area_px").as_int();
      detector_ = std::make_unique<HsvDetector>(std::move(cfg));
      RCLCPP_INFO(get_logger(), "Detection backend: HSV");
    } else if (backend_ == "yolo") {
#ifdef USE_ONNXRUNTIME
      YoloV11Config cfg;
      cfg.model_path = get_parameter("model_path").as_string();
      cfg.use_cuda = get_parameter("use_cuda").as_bool();
      cfg.conf_threshold = static_cast<float>(
          get_parameter("conf_threshold").as_double());
      cfg.iou_threshold = static_cast<float>(
          get_parameter("iou_threshold").as_double());
      detector_ = std::make_unique<YoloV11>(std::move(cfg));
      RCLCPP_INFO(get_logger(), "Detection backend: YOLOv11 (ONNX)");
#else
      throw std::runtime_error(
          "backend=yolo requested but perception was built without "
          "ONNXRuntime support");
#endif
    } else {
      throw std::runtime_error("Unknown detection backend: " + backend_);
    }

    pub_ = create_publisher<perception_bot_msgs::msg::DetectedObjectArray>(
        output_topic_, 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          on_image(std::move(msg));
        });

    RCLCPP_INFO(get_logger(),
        "detection_node ready: %s → %s",
        image_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    perception_bot_msgs::msg::DetectedObjectArray out;
    out.header = msg->header;
    out.objects = detector_->detect(cv_ptr->image, msg->header);

    pub_->publish(out);
  }

  std::string backend_;
  std::string image_topic_;
  std::string output_topic_;
  std::unique_ptr<Detector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<perception_bot_msgs::msg::DetectedObjectArray>::SharedPtr
      pub_;
};

}  // namespace perception_bot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_bot::DetectionNode>());
  rclcpp::shutdown();
  return 0;
}
