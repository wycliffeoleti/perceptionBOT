// Color + shape detector for the sorting demo.
//   red / blue / green via HSV thresholding
//   cube vs cylinder via contour aspect ratio
#pragma once

#include "perception_bot_perception/detector.hpp"

#include <opencv2/core.hpp>

namespace perception_bot {

struct HsvRange {
  cv::Scalar lo;
  cv::Scalar hi;
};

struct HsvDetectorConfig {
  // Two ranges for red because hue wraps at 0/180.
  HsvRange red_lo{cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255)};
  HsvRange red_hi{cv::Scalar(170, 100, 80), cv::Scalar(180, 255, 255)};
  HsvRange blue{cv::Scalar(100, 120, 60), cv::Scalar(130, 255, 255)};
  HsvRange green{cv::Scalar(45, 80, 60), cv::Scalar(85, 255, 255)};

  int min_area_px = 200;       // ignore blobs smaller than this
  int morph_kernel = 5;        // open/close kernel size
  // Aspect-ratio threshold: |1 - bbox_w/bbox_h| < cube_aspect_tol → "cube"; else "cylinder".
  double cube_aspect_tol = 0.25;
};

class HsvDetector final : public Detector {
public:
  explicit HsvDetector(HsvDetectorConfig cfg = {});

  std::vector<perception_bot_msgs::msg::DetectedObject>
  detect(const cv::Mat &bgr_image, const std_msgs::msg::Header &header) override;

private:
  HsvDetectorConfig cfg_;
};

}  // namespace perception_bot
