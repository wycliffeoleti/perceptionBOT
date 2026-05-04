// pose_estimation_node — fuse 2D detections with depth + camera intrinsics
// to produce 6DoF object poses in the world frame.
//
// Subscriptions (synchronized):
//   /perception/detections      perception_bot_msgs/DetectedObjectArray
//   <depth topic>               sensor_msgs/Image  (CV_32FC1, meters)
//   /camera/camera_info         sensor_msgs/CameraInfo
// Publishes:
//   /perception/objects_world   perception_bot_msgs/DetectedObjectArray
//
// The pose orientation is a top-down grasp: camera_optical z+ points into
// the scene, but in the world frame we set rotation so that the gripper's
// approach axis is -Z (top-down). For the FR3 hand frame the convention is
// that the TCP +Z points away from the wrist, so a top-down grasp pose has
// orientation = quaternion(roll=pi, pitch=0, yaw=θ) where θ is the object's
// principal axis around world Z. We use yaw=0 for cubes and the bbox aspect
// for cylinders (axis-aligned approximation).
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <perception_bot_msgs/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace perception_bot {

class PoseEstimationNode : public rclcpp::Node {
public:
  PoseEstimationNode() : rclcpp::Node("pose_estimation_node") {
    detections_topic_ = declare_parameter<std::string>(
        "detections_topic", "/perception/detections");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/perception/depth_gt");
    camera_info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "/camera/camera_info");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/perception/objects_world");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    sync_slop_ = declare_parameter<double>("sync_slop_seconds", 0.1);
    default_height_m_ = declare_parameter<double>("default_object_height_m", 0.05);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<perception_bot_msgs::msg::DetectedObjectArray>(
        output_topic_, 10);

    auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    sub_det_.subscribe(this, detections_topic_, qos);
    sub_depth_.subscribe(this, depth_topic_, qos);
    sub_info_.subscribe(this, camera_info_topic_, qos);

    sync_ = std::make_shared<Sync>(SyncPolicy(20),
                                   sub_det_, sub_depth_, sub_info_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_));
    sync_->registerCallback(std::bind(&PoseEstimationNode::on_synced, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));

    RCLCPP_INFO(get_logger(),
        "pose_estimation_node ready: %s + %s + %s → %s (world=%s)",
        detections_topic_.c_str(), depth_topic_.c_str(),
        camera_info_topic_.c_str(), output_topic_.c_str(), world_frame_.c_str());
  }

private:
  using DetMsg = perception_bot_msgs::msg::DetectedObjectArray;
  using ImgMsg = sensor_msgs::msg::Image;
  using InfoMsg = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      DetMsg, ImgMsg, InfoMsg>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // Median of finite, positive depths inside the bbox.
  static double median_depth(const cv::Mat &depth, const cv::Rect &roi) {
    cv::Rect clipped = roi & cv::Rect(0, 0, depth.cols, depth.rows);
    if (clipped.area() == 0) return 0.0;
    std::vector<float> samples;
    samples.reserve(clipped.area());
    for (int y = clipped.y; y < clipped.y + clipped.height; ++y) {
      const float *p = depth.ptr<float>(y);
      for (int x = clipped.x; x < clipped.x + clipped.width; ++x) {
        float d = p[x];
        if (std::isfinite(d) && d > 0.05f && d < 5.0f) samples.push_back(d);
      }
    }
    if (samples.empty()) return 0.0;
    auto mid = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), mid, samples.end());
    return *mid;
  }

  static double dim_for_class(const std::string &cls, double bbox_w_m,
                              double bbox_h_m, double default_h) {
    // Returns object height (z-extent in world frame).
    if (cls.find("cube") != std::string::npos) {
      return std::max(bbox_w_m, bbox_h_m);
    }
    if (cls.find("cylinder") != std::string::npos) {
      return std::max(0.05, bbox_h_m);
    }
    return default_h;
  }

  void on_synced(const DetMsg::ConstSharedPtr &det,
                 const ImgMsg::ConstSharedPtr &depth_msg,
                 const InfoMsg::ConstSharedPtr &info) {
    cv_bridge::CvImageConstPtr depth_cv;
    try {
      depth_cv = cv_bridge::toCvShare(depth_msg, "32FC1");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge depth: %s", e.what());
      return;
    }

    image_geometry::PinholeCameraModel cam;
    cam.fromCameraInfo(*info);

    geometry_msgs::msg::TransformStamped tf_world_cam;
    try {
      tf_world_cam = tf_buffer_->lookupTransform(
          world_frame_, det->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "tf2 lookup failed: %s", ex.what());
      return;
    }

    DetMsg out;
    out.header.stamp = det->header.stamp;
    out.header.frame_id = world_frame_;
    out.objects.reserve(det->objects.size());

    for (const auto &obj : det->objects) {
      cv::Rect roi(static_cast<int>(obj.bbox.x_offset),
                   static_cast<int>(obj.bbox.y_offset),
                   static_cast<int>(obj.bbox.width),
                   static_cast<int>(obj.bbox.height));
      double z = median_depth(depth_cv->image, roi);
      if (z <= 0.0) {
        RCLCPP_DEBUG(get_logger(), "skip %s: no valid depth in bbox",
            obj.class_name.c_str());
        continue;
      }
      double cx = obj.bbox.x_offset + obj.bbox.width * 0.5;
      double cy = obj.bbox.y_offset + obj.bbox.height * 0.5;
      cv::Point3d ray = cam.projectPixelTo3dRay(cv::Point2d(cx, cy));
      double norm = std::sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
      if (norm < 1e-9) continue;
      cv::Point3d cam_pt(ray.x * z / ray.z, ray.y * z / ray.z, z);

      // Real-world bbox dimensions in meters at depth z (small-angle).
      double w_m = obj.bbox.width  * z / cam.fx();
      double h_m = obj.bbox.height * z / cam.fy();
      double obj_h = dim_for_class(obj.class_name, w_m, h_m, default_height_m_);

      geometry_msgs::msg::PoseStamped p_cam;
      p_cam.header.stamp = det->header.stamp;
      p_cam.header.frame_id = det->header.frame_id;
      p_cam.pose.position.x = cam_pt.x;
      p_cam.pose.position.y = cam_pt.y;
      p_cam.pose.position.z = cam_pt.z;
      p_cam.pose.orientation.w = 1.0;

      geometry_msgs::msg::PoseStamped p_world;
      tf2::doTransform(p_cam, p_world, tf_world_cam);

      // Top-down grasp: roll=π so the gripper z-axis points -Z in world.
      tf2::Quaternion q;
      q.setRPY(M_PI, 0.0, 0.0);
      p_world.pose.orientation = tf2::toMsg(q);

      perception_bot_msgs::msg::DetectedObject world_obj = obj;
      world_obj.header = out.header;
      world_obj.pose = p_world;
      world_obj.dimensions[0] = static_cast<float>(w_m);
      world_obj.dimensions[1] = static_cast<float>(w_m);
      world_obj.dimensions[2] = static_cast<float>(obj_h);
      out.objects.push_back(std::move(world_obj));
    }

    pub_->publish(out);
  }

  std::string detections_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string world_frame_;
  double sync_slop_;
  double default_height_m_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<DetMsg> sub_det_;
  message_filters::Subscriber<ImgMsg> sub_depth_;
  message_filters::Subscriber<InfoMsg> sub_info_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<DetMsg>::SharedPtr pub_;
};

}  // namespace perception_bot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_bot::PoseEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
