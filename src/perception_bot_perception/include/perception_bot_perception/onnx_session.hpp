// RAII wrapper around an ONNX Runtime session with CUDA EP fallback to CPU.
//
// Header is only compiled when USE_ONNXRUNTIME is defined (set by CMake when
// the onnxruntime headers/library are found).
#pragma once

#ifdef USE_ONNXRUNTIME

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace perception_bot {

class OnnxSession {
public:
  OnnxSession(const std::string &model_path, bool prefer_cuda) {
    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    opts.SetIntraOpNumThreads(1);

    if (prefer_cuda) {
      try {
        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = 0;
        opts.AppendExecutionProvider_CUDA(cuda_opts);
        used_cuda_ = true;
      } catch (const std::exception &e) {
        used_cuda_ = false;
      }
    }

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts);

    Ort::AllocatorWithDefaultOptions alloc;
    for (size_t i = 0; i < session_->GetInputCount(); ++i) {
      auto name = session_->GetInputNameAllocated(i, alloc);
      input_names_.emplace_back(name.get());
    }
    for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
      auto name = session_->GetOutputNameAllocated(i, alloc);
      output_names_.emplace_back(name.get());
    }
  }

  Ort::Session &session() { return *session_; }
  const std::vector<std::string> &input_names() const { return input_names_; }
  const std::vector<std::string> &output_names() const { return output_names_; }
  bool used_cuda() const { return used_cuda_; }

  // Helper that returns C-string arrays for Run().
  std::vector<const char *> input_names_c() const {
    std::vector<const char *> v;
    v.reserve(input_names_.size());
    for (auto &s : input_names_) v.push_back(s.c_str());
    return v;
  }
  std::vector<const char *> output_names_c() const {
    std::vector<const char *> v;
    v.reserve(output_names_.size());
    for (auto &s : output_names_) v.push_back(s.c_str());
    return v;
  }

private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "perception_bot"};
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  bool used_cuda_{false};
};

}  // namespace perception_bot

#endif  // USE_ONNXRUNTIME
