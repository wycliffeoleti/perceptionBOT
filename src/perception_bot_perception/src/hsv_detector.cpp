#include "perception_bot_perception/hsv_detector.hpp"

#include <opencv2/imgproc.hpp>

namespace perception_bot {

namespace {

cv::Mat make_mask(const cv::Mat &hsv, const HsvRange &r) {
  cv::Mat m;
  cv::inRange(hsv, r.lo, r.hi, m);
  return m;
}

void morph(cv::Mat &mask, int k) {
  if (k <= 1) return;
  auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
}

void emit_detections(
    const cv::Mat &mask,
    const std::string &color,
    const HsvDetectorConfig &cfg,
    const std_msgs::msg::Header &header,
    std::vector<perception_bot_msgs::msg::DetectedObject> &out) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto &c : contours) {
    double area = cv::contourArea(c);
    if (area < cfg.min_area_px) continue;

    cv::Rect bbox = cv::boundingRect(c);
    double w = static_cast<double>(bbox.width);
    double h = static_cast<double>(bbox.height);
    double aspect = (h > 0.0) ? w / h : 0.0;

    std::string shape;
    // Green is always cylinder in our world; classification stays for robustness.
    if (color == "green") {
      shape = (std::abs(1.0 - aspect) < cfg.cube_aspect_tol) ? "cube" : "cylinder";
    } else {
      shape = (std::abs(1.0 - aspect) < cfg.cube_aspect_tol) ? "cube" : "cylinder";
    }

    perception_bot_msgs::msg::DetectedObject obj;
    obj.header = header;
    obj.class_name = color + "_" + shape;
    // Confidence proxy: how "filled" the bbox is by the mask.
    double bbox_area = w * h;
    obj.confidence = static_cast<float>(bbox_area > 0 ? area / bbox_area : 0.0);
    obj.bbox.x_offset = static_cast<uint32_t>(std::max(0, bbox.x));
    obj.bbox.y_offset = static_cast<uint32_t>(std::max(0, bbox.y));
    obj.bbox.width = static_cast<uint32_t>(bbox.width);
    obj.bbox.height = static_cast<uint32_t>(bbox.height);
    obj.bbox.do_rectify = false;
    obj.dimensions = {0.0f, 0.0f, 0.0f};  // pose node fills these
    obj.pose.header = header;
    out.push_back(std::move(obj));
  }
}

}  // namespace

HsvDetector::HsvDetector(HsvDetectorConfig cfg) : cfg_(std::move(cfg)) {}

std::vector<perception_bot_msgs::msg::DetectedObject>
HsvDetector::detect(const cv::Mat &bgr, const std_msgs::msg::Header &header) {
  std::vector<perception_bot_msgs::msg::DetectedObject> out;
  if (bgr.empty()) return out;

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat red_mask = make_mask(hsv, cfg_.red_lo);
  red_mask |= make_mask(hsv, cfg_.red_hi);
  cv::Mat blue_mask = make_mask(hsv, cfg_.blue);
  cv::Mat green_mask = make_mask(hsv, cfg_.green);

  morph(red_mask, cfg_.morph_kernel);
  morph(blue_mask, cfg_.morph_kernel);
  morph(green_mask, cfg_.morph_kernel);

  emit_detections(red_mask, "red", cfg_, header, out);
  emit_detections(blue_mask, "blue", cfg_, header, out);
  emit_detections(green_mask, "green", cfg_, header, out);

  return out;
}

}  // namespace perception_bot
