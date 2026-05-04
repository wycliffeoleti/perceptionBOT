#ifdef USE_ONNXRUNTIME

#include "perception_bot_perception/depth_anything.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace perception_bot {

namespace {

// ImageNet normalization used by DA-V2.
constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};
constexpr float kStd[3] = {0.229f, 0.224f, 0.225f};

}  // namespace

DepthAnything::DepthAnything(DepthAnythingConfig cfg) : cfg_(std::move(cfg)) {
  if (cfg_.model_path.empty()) {
    throw std::runtime_error("DepthAnything: empty model_path");
  }
  sess_ = std::make_unique<OnnxSession>(cfg_.model_path, cfg_.use_cuda);
}

cv::Mat DepthAnything::raw_inference(const cv::Mat &bgr) {
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(cfg_.input_size, cfg_.input_size));
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
  std::vector<cv::Mat> chw(3);
  cv::split(rgb, chw);
  for (int c = 0; c < 3; ++c) {
    chw[c] = (chw[c] - kMean[c]) / kStd[c];
  }

  std::vector<float> input;
  input.reserve(static_cast<size_t>(cfg_.input_size) * cfg_.input_size * 3);
  for (int c = 0; c < 3; ++c) {
    input.insert(input.end(), chw[c].begin<float>(), chw[c].end<float>());
  }

  std::array<int64_t, 4> shape = {1, 3, cfg_.input_size, cfg_.input_size};
  Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value t = Ort::Value::CreateTensor<float>(
      mem, input.data(), input.size(), shape.data(), shape.size());

  auto in_names = sess_->input_names_c();
  auto out_names = sess_->output_names_c();
  auto outs = sess_->session().Run(
      Ort::RunOptions{nullptr},
      in_names.data(), &t, 1,
      out_names.data(), out_names.size());

  auto info = outs[0].GetTensorTypeAndShapeInfo();
  auto oshape = info.GetShape();
  // Expect [1, H, W] or [1, 1, H, W]. Some exports use [H, W].
  int H = cfg_.input_size, W = cfg_.input_size;
  if (oshape.size() == 4) {
    H = static_cast<int>(oshape[2]);
    W = static_cast<int>(oshape[3]);
  } else if (oshape.size() == 3) {
    H = static_cast<int>(oshape[1]);
    W = static_cast<int>(oshape[2]);
  } else if (oshape.size() == 2) {
    H = static_cast<int>(oshape[0]);
    W = static_cast<int>(oshape[1]);
  }

  cv::Mat inv_depth(H, W, CV_32FC1, const_cast<float *>(outs[0].GetTensorData<float>()));
  cv::Mat copy = inv_depth.clone();

  cv::Mat out;
  cv::resize(copy, out, cv::Size(bgr.cols, bgr.rows), 0, 0, cv::INTER_LINEAR);
  return out;
}

cv::Mat DepthAnything::infer(const cv::Mat &bgr) {
  cv::Mat inv = raw_inference(bgr);
  if (!scale_.has_value()) {
    return inv;
  }

  cv::Mat metric(inv.size(), CV_32FC1);
  const float s = *scale_;
  for (int y = 0; y < inv.rows; ++y) {
    const float *src = inv.ptr<float>(y);
    float *dst = metric.ptr<float>(y);
    for (int x = 0; x < inv.cols; ++x) {
      dst[x] = (src[x] > 1e-6f) ? s / src[x] : 0.f;
    }
  }
  return metric;
}

void DepthAnything::calibrate_against_gt(const cv::Mat &bgr,
                                        const cv::Mat &gt_depth) {
  if (gt_depth.type() != CV_32FC1 || gt_depth.empty()) {
    throw std::runtime_error(
        "DepthAnything::calibrate: gt_depth must be CV_32FC1 (meters)");
  }
  cv::Mat inv = raw_inference(bgr);

  cv::Mat gt;
  if (gt_depth.size() != inv.size()) {
    cv::resize(gt_depth, gt, inv.size(), 0, 0, cv::INTER_NEAREST);
  } else {
    gt = gt_depth;
  }

  // Collect ratios scale = gt * inv_depth (so metric = scale / inv_depth).
  std::vector<float> ratios;
  ratios.reserve(gt.total() / 8);
  for (int y = 0; y < gt.rows; ++y) {
    const float *gtp = gt.ptr<float>(y);
    const float *invp = inv.ptr<float>(y);
    for (int x = 0; x < gt.cols; ++x) {
      float d = gtp[x];
      float i = invp[x];
      if (std::isfinite(d) && d > 0.05f && d < 5.0f && i > 1e-6f) {
        ratios.push_back(d * i);
      }
    }
  }

  if (ratios.size() < 100) {
    throw std::runtime_error(
        "DepthAnything::calibrate: not enough valid GT depth pixels");
  }
  std::nth_element(ratios.begin(),
                   ratios.begin() + ratios.size() / 2,
                   ratios.end());
  scale_ = ratios[ratios.size() / 2];
}

}  // namespace perception_bot

#endif  // USE_ONNXRUNTIME
