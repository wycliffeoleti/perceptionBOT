// YOLOv11n ONNX inference + COCO-class detection.
//
// Only compiled when USE_ONNXRUNTIME is defined.
#pragma once

#ifdef USE_ONNXRUNTIME

#include "perception_bot_perception/detector.hpp"
#include "perception_bot_perception/onnx_session.hpp"

#include <memory>
#include <string>
#include <vector>

namespace perception_bot {

struct YoloV11Config {
  std::string model_path;       // path to yolov11n.onnx
  bool use_cuda = true;
  int input_size = 640;         // square letterbox
  float conf_threshold = 0.25f;
  float iou_threshold = 0.45f;
  int max_detections = 100;
};

class YoloV11 final : public Detector {
public:
  explicit YoloV11(YoloV11Config cfg);

  std::vector<perception_bot_msgs::msg::DetectedObject>
  detect(const cv::Mat &bgr_image, const std_msgs::msg::Header &header) override;

private:
  YoloV11Config cfg_;
  std::unique_ptr<OnnxSession> sess_;
  std::vector<std::string> class_names_;  // COCO 80 classes
};

}  // namespace perception_bot

#endif  // USE_ONNXRUNTIME
