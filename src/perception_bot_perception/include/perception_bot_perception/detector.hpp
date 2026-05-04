// Abstract detector interface — both HSV and YOLOv11 backends implement this.
#pragma once

#include <opencv2/core.hpp>
#include <perception_bot_msgs/msg/detected_object.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

namespace perception_bot {

class Detector {
public:
  virtual ~Detector() = default;

  // bgr_image is HxWxC uint8 BGR. Returns detections with bbox in pixel coords
  // and class_name set. pose/dimensions are left empty (filled by pose node).
  virtual std::vector<perception_bot_msgs::msg::DetectedObject>
  detect(const cv::Mat &bgr_image, const std_msgs::msg::Header &header) = 0;
};

}  // namespace perception_bot
