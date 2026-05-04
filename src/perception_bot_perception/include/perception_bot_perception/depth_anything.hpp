// Depth Anything V2 ViT-S ONNX inference + metric calibration.
//
// Only compiled when USE_ONNXRUNTIME is defined.
//
// Output of the public ONNX export is a relative inverse-depth map that we
// align to metric using a one-time scale-and-shift fit against a single
// Gazebo ground-truth depth frame (median-of-ratios). After calibration the
// node returns a metric depth image (32FC1, meters) with the same intrinsics
// as the input camera.
#pragma once

#ifdef USE_ONNXRUNTIME

#include "perception_bot_perception/onnx_session.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <optional>
#include <string>

namespace perception_bot {

struct DepthAnythingConfig {
  std::string model_path;
  bool use_cuda = true;
  int input_size = 518;  // DA-V2 ViT-S
};

class DepthAnything {
public:
  explicit DepthAnything(DepthAnythingConfig cfg);

  // Run inference on bgr_image, return a CV_32FC1 metric-depth image at the
  // input image resolution. If no calibration has been provided yet, this
  // returns the raw inverse-depth output (still 32FC1, but in arbitrary units).
  cv::Mat infer(const cv::Mat &bgr_image);

  // Calibrate metric scale by aligning a single inference against a Gazebo
  // ground-truth depth frame (CV_32FC1 meters). Uses the median ratio of
  // (gt_depth) / (1 / inv_depth) over valid pixels.
  void calibrate_against_gt(const cv::Mat &bgr_image, const cv::Mat &gt_depth);

  bool is_calibrated() const { return scale_.has_value(); }

private:
  cv::Mat raw_inference(const cv::Mat &bgr_image);  // returns 32FC1 inv-depth

  DepthAnythingConfig cfg_;
  std::unique_ptr<OnnxSession> sess_;
  std::optional<float> scale_;  // metric_depth = scale_ / inv_depth
};

}  // namespace perception_bot

#endif  // USE_ONNXRUNTIME
