#ifdef USE_ONNXRUNTIME

#include "perception_bot_perception/yolo_v11.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace perception_bot {

namespace {

constexpr std::array<const char *, 80> kCocoClasses = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
};

struct Letterbox {
  cv::Mat image;
  float scale;
  int pad_x;
  int pad_y;
};

Letterbox letterbox(const cv::Mat &src, int target) {
  float r = std::min(static_cast<float>(target) / src.cols,
                     static_cast<float>(target) / src.rows);
  int new_w = static_cast<int>(std::round(src.cols * r));
  int new_h = static_cast<int>(std::round(src.rows * r));
  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h));
  int pad_x = (target - new_w) / 2;
  int pad_y = (target - new_h) / 2;
  cv::Mat out(target, target, src.type(), cv::Scalar(114, 114, 114));
  resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));
  return {out, r, pad_x, pad_y};
}

}  // namespace

YoloV11::YoloV11(YoloV11Config cfg) : cfg_(std::move(cfg)) {
  if (cfg_.model_path.empty()) {
    throw std::runtime_error("YoloV11: empty model_path");
  }
  sess_ = std::make_unique<OnnxSession>(cfg_.model_path, cfg_.use_cuda);
  class_names_.assign(kCocoClasses.begin(), kCocoClasses.end());
}

std::vector<perception_bot_msgs::msg::DetectedObject>
YoloV11::detect(const cv::Mat &bgr, const std_msgs::msg::Header &header) {
  std::vector<perception_bot_msgs::msg::DetectedObject> out;
  if (bgr.empty()) return out;

  // Preprocess: letterbox → BGR→RGB → /255 → CHW float32.
  Letterbox lb = letterbox(bgr, cfg_.input_size);
  cv::Mat rgb;
  cv::cvtColor(lb.image, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
  std::vector<cv::Mat> chw_planes(3);
  cv::split(rgb, chw_planes);

  std::vector<float> input_tensor_values;
  input_tensor_values.reserve(static_cast<size_t>(cfg_.input_size) *
                              cfg_.input_size * 3);
  for (int c = 0; c < 3; ++c) {
    input_tensor_values.insert(
        input_tensor_values.end(),
        chw_planes[c].begin<float>(), chw_planes[c].end<float>());
  }

  std::array<int64_t, 4> input_shape = {1, 3, cfg_.input_size, cfg_.input_size};
  Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input = Ort::Value::CreateTensor<float>(
      mem, input_tensor_values.data(), input_tensor_values.size(),
      input_shape.data(), input_shape.size());

  auto in_names = sess_->input_names_c();
  auto out_names = sess_->output_names_c();
  auto outputs = sess_->session().Run(
      Ort::RunOptions{nullptr},
      in_names.data(), &input, 1,
      out_names.data(), out_names.size());

  // YOLOv11 output: [1, 84, N] where N≈8400 for 640. 84 = 4 box + 80 cls.
  auto info = outputs[0].GetTensorTypeAndShapeInfo();
  auto shape = info.GetShape();
  if (shape.size() != 3 || shape[1] != 84) {
    return out;
  }
  const float *data = outputs[0].GetTensorData<float>();
  const int64_t channels = shape[1];  // 84
  const int64_t num = shape[2];       // ≈8400

  std::vector<cv::Rect2f> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  boxes.reserve(num);
  scores.reserve(num);
  classes.reserve(num);

  for (int64_t i = 0; i < num; ++i) {
    int best_cls = -1;
    float best_score = 0.f;
    for (int c = 4; c < channels; ++c) {
      float s = data[c * num + i];
      if (s > best_score) {
        best_score = s;
        best_cls = c - 4;
      }
    }
    if (best_score < cfg_.conf_threshold) continue;

    float cx = data[0 * num + i];
    float cy = data[1 * num + i];
    float w = data[2 * num + i];
    float h = data[3 * num + i];
    float x = cx - w * 0.5f;
    float y = cy - h * 0.5f;
    boxes.emplace_back(x, y, w, h);
    scores.push_back(best_score);
    classes.push_back(best_cls);
  }

  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, cfg_.conf_threshold, cfg_.iou_threshold,
                    keep, 1.f, cfg_.max_detections);

  for (int idx : keep) {
    const auto &b = boxes[idx];
    // Reverse letterbox.
    float x = (b.x - lb.pad_x) / lb.scale;
    float y = (b.y - lb.pad_y) / lb.scale;
    float w = b.width / lb.scale;
    float h = b.height / lb.scale;
    x = std::max(0.f, x);
    y = std::max(0.f, y);
    w = std::min(static_cast<float>(bgr.cols) - x, w);
    h = std::min(static_cast<float>(bgr.rows) - y, h);
    if (w <= 0 || h <= 0) continue;

    perception_bot_msgs::msg::DetectedObject obj;
    obj.header = header;
    obj.class_name = class_names_[classes[idx]];
    obj.confidence = scores[idx];
    obj.bbox.x_offset = static_cast<uint32_t>(x);
    obj.bbox.y_offset = static_cast<uint32_t>(y);
    obj.bbox.width = static_cast<uint32_t>(w);
    obj.bbox.height = static_cast<uint32_t>(h);
    obj.bbox.do_rectify = false;
    obj.dimensions = {0.f, 0.f, 0.f};
    obj.pose.header = header;
    out.push_back(std::move(obj));
  }

  return out;
}

}  // namespace perception_bot

#endif  // USE_ONNXRUNTIME
