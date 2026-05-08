#include "perception_inference/detector_backend.hpp"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace perception_inference
{
namespace
{

constexpr int kDefaultNetworkSize = 640;
constexpr int kMaxDetections = 100;

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING && msg != nullptr) {
      last_message_ = msg;
    }
  }

  std::string last_message() const { return last_message_; }

private:
  std::string last_message_;
};

TensorRtLogger& trt_logger()
{
  static TensorRtLogger logger;
  return logger;
}

void check_cuda(cudaError_t status, const std::string& context)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(context + ": " + cudaGetErrorString(status));
  }
}

std::size_t data_type_size(nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT: return 4;
    case nvinfer1::DataType::kHALF: return 2;
    case nvinfer1::DataType::kINT8: return 1;
    case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kINT64: return 8;
    case nvinfer1::DataType::kBOOL: return 1;
    case nvinfer1::DataType::kUINT8: return 1;
    case nvinfer1::DataType::kFP8: return 1;
    case nvinfer1::DataType::kBF16: return 2;
    case nvinfer1::DataType::kINT4: return 1;
  }
  return 0;
}

std::size_t volume(const nvinfer1::Dims& dims)
{
  if (dims.nbDims <= 0) {
    return 0;
  }
  std::size_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    count *= static_cast<std::size_t>(dims.d[i]);
  }
  return count;
}

std::string dims_to_string(const nvinfer1::Dims& dims)
{
  std::ostringstream out;
  out << "[";
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i > 0) {
      out << "x";
    }
    out << dims.d[i];
  }
  out << "]";
  return out.str();
}

struct DeviceBuffer
{
  ~DeviceBuffer()
  {
    if (ptr != nullptr) {
      cudaFree(ptr);
    }
  }

  void resize(std::size_t bytes)
  {
    if (bytes <= size_bytes) {
      return;
    }
    if (ptr != nullptr) {
      check_cuda(cudaFree(ptr), "cudaFree");
      ptr = nullptr;
      size_bytes = 0;
    }
    check_cuda(cudaMalloc(&ptr, bytes), "cudaMalloc");
    size_bytes = bytes;
  }

  void* ptr{nullptr};
  std::size_t size_bytes{0};
};

struct CudaStream
{
  CudaStream()
  {
    check_cuda(cudaStreamCreate(&stream), "cudaStreamCreate");
  }

  ~CudaStream()
  {
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
    }
  }

  cudaStream_t stream{nullptr};
};

class DebugDetectorBackend final : public DetectorBackend
{
public:
  explicit DebugDetectorBackend(DetectorConfig config) : config_(std::move(config)) {}

  std::string id() const override { return config_.model_name; }
  bool ready() const override { return true; }
  std::string status() const override { return "debug backend active"; }

  std::vector<amr_interfaces::msg::Detection2D> detect(const sensor_msgs::msg::Image& image) override
  {
    if (!config_.publish_debug_detections) {
      return {};
    }

    amr_interfaces::msg::Detection2D detection;
    detection.detector_id = config_.model_name;
    detection.class_id = "person";
    detection.class_index = 0;
    detection.score = 0.90f;
    detection.center_x = static_cast<float>(image.width) * 0.5f;
    detection.center_y = static_cast<float>(image.height) * 0.55f;
    detection.size_x = static_cast<float>(image.width) * 0.18f;
    detection.size_y = static_cast<float>(image.height) * 0.35f;
    return {detection};
  }

private:
  DetectorConfig config_;
};

struct Letterbox
{
  int input_w{kDefaultNetworkSize};
  int input_h{kDefaultNetworkSize};
  float scale{1.0f};
  float pad_x{0.0f};
  float pad_y{0.0f};
};

float read_channel(const sensor_msgs::msg::Image& image, int x, int y, int channel)
{
  const auto* row = image.data.data() + static_cast<std::size_t>(y) * image.step;
  const auto* pixel = row + static_cast<std::size_t>(x) * 3;
  const bool rgb = image.encoding == "rgb8";
  const bool bgr = image.encoding == "bgr8";

  if (!rgb && !bgr) {
    return 0.0f;
  }

  if (rgb) {
    return static_cast<float>(pixel[channel]) / 255.0f;
  }

  const int bgr_index = channel == 0 ? 2 : (channel == 1 ? 1 : 0);
  return static_cast<float>(pixel[bgr_index]) / 255.0f;
}

void preprocess_letterbox(
  const sensor_msgs::msg::Image& image,
  int input_w,
  int input_h,
  std::vector<float>& output,
  Letterbox& letterbox)
{
  if (image.encoding != "bgr8" && image.encoding != "rgb8") {
    throw std::runtime_error("YOLO TensorRT backend only supports bgr8/rgb8 images; got " + image.encoding);
  }

  output.assign(static_cast<std::size_t>(3 * input_h * input_w), 114.0f / 255.0f);

  const auto src_w = static_cast<float>(image.width);
  const auto src_h = static_cast<float>(image.height);
  const float scale = std::min(static_cast<float>(input_w) / src_w, static_cast<float>(input_h) / src_h);
  const int resized_w = static_cast<int>(std::round(src_w * scale));
  const int resized_h = static_cast<int>(std::round(src_h * scale));
  const float pad_x = static_cast<float>(input_w - resized_w) * 0.5f;
  const float pad_y = static_cast<float>(input_h - resized_h) * 0.5f;

  for (int y = 0; y < resized_h; ++y) {
    const float src_y = (static_cast<float>(y) + 0.5f) / scale - 0.5f;
    const int y0 = std::clamp(static_cast<int>(std::floor(src_y)), 0, static_cast<int>(image.height) - 1);
    const int y1 = std::min(y0 + 1, static_cast<int>(image.height) - 1);
    const float wy = src_y - static_cast<float>(y0);

    for (int x = 0; x < resized_w; ++x) {
      const float src_x = (static_cast<float>(x) + 0.5f) / scale - 0.5f;
      const int x0 = std::clamp(static_cast<int>(std::floor(src_x)), 0, static_cast<int>(image.width) - 1);
      const int x1 = std::min(x0 + 1, static_cast<int>(image.width) - 1);
      const float wx = src_x - static_cast<float>(x0);
      const int dst_x = static_cast<int>(pad_x) + x;
      const int dst_y = static_cast<int>(pad_y) + y;

      for (int c = 0; c < 3; ++c) {
        const float v00 = read_channel(image, x0, y0, c);
        const float v01 = read_channel(image, x1, y0, c);
        const float v10 = read_channel(image, x0, y1, c);
        const float v11 = read_channel(image, x1, y1, c);
        const float top = v00 * (1.0f - wx) + v01 * wx;
        const float bottom = v10 * (1.0f - wx) + v11 * wx;
        output[static_cast<std::size_t>(c * input_h * input_w + dst_y * input_w + dst_x)] =
          top * (1.0f - wy) + bottom * wy;
      }
    }
  }

  letterbox.input_w = input_w;
  letterbox.input_h = input_h;
  letterbox.scale = scale;
  letterbox.pad_x = pad_x;
  letterbox.pad_y = pad_y;
}

struct Candidate
{
  int class_index{0};
  float score{0.0f};
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
};

float iou(const Candidate& a, const Candidate& b)
{
  const float xx1 = std::max(a.x1, b.x1);
  const float yy1 = std::max(a.y1, b.y1);
  const float xx2 = std::min(a.x2, b.x2);
  const float yy2 = std::min(a.y2, b.y2);
  const float w = std::max(0.0f, xx2 - xx1);
  const float h = std::max(0.0f, yy2 - yy1);
  const float inter = w * h;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float denom = area_a + area_b - inter;
  return denom <= 0.0f ? 0.0f : inter / denom;
}

std::vector<Candidate> nms(std::vector<Candidate> candidates, float iou_threshold)
{
  std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.score > rhs.score;
  });

  std::vector<Candidate> kept;
  kept.reserve(std::min<std::size_t>(candidates.size(), kMaxDetections));
  std::vector<bool> removed(candidates.size(), false);

  for (std::size_t i = 0; i < candidates.size() && kept.size() < kMaxDetections; ++i) {
    if (removed[i]) {
      continue;
    }
    kept.push_back(candidates[i]);
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
      if (!removed[j] && candidates[i].class_index == candidates[j].class_index &&
          iou(candidates[i], candidates[j]) > iou_threshold) {
        removed[j] = true;
      }
    }
  }

  return kept;
}

class TensorRtYoloBackend final : public DetectorBackend
{
public:
  TensorRtYoloBackend(DetectorConfig config, std::vector<std::string> labels)
  : config_(std::move(config)), labels_(std::move(labels))
  {
    try {
      initLibNvInferPlugins(&trt_logger(), "");
      load_engine();
      status_ = config_.backend + " engine ready: " + config_.engine_path;
      ready_ = true;
    } catch (const std::exception& ex) {
      status_ = ex.what();
      ready_ = false;
    }
  }

  std::string id() const override { return config_.model_name; }
  bool ready() const override { return ready_; }
  std::string status() const override { return status_; }

  std::vector<amr_interfaces::msg::Detection2D> detect(const sensor_msgs::msg::Image& image) override
  {
    if (!ready_) {
      return {};
    }

    try {
      Letterbox letterbox;
      preprocess_letterbox(image, input_w_, input_h_, input_float_, letterbox);
      upload_input();

      if (!context_->enqueueV3(stream_.stream)) {
        status_ = "TensorRT enqueueV3 failed";
        return {};
      }

      check_cuda(
        cudaMemcpyAsync(
          output_host_bytes_.data(),
          output_buffer_.ptr,
          output_host_bytes_.size(),
          cudaMemcpyDeviceToHost,
          stream_.stream),
        "cudaMemcpyAsync output");
      check_cuda(cudaStreamSynchronize(stream_.stream), "cudaStreamSynchronize");

      auto candidates = decode(letterbox, static_cast<float>(image.width), static_cast<float>(image.height));
      auto kept = nms(std::move(candidates), 0.45f);
      return to_messages(kept);
    } catch (const std::exception& ex) {
      status_ = std::string("TensorRT inference failed: ") + ex.what();
      return {};
    }
  }

private:
  void load_engine()
  {
    if (config_.engine_path.empty() || !std::filesystem::exists(config_.engine_path)) {
      throw std::runtime_error("TensorRT engine is not configured or does not exist: " + config_.engine_path);
    }

    std::ifstream file(config_.engine_path, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("failed to open TensorRT engine: " + config_.engine_path);
    }
    const auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(static_cast<std::size_t>(size));
    if (!file.read(engine_data.data(), size)) {
      throw std::runtime_error("failed to read TensorRT engine: " + config_.engine_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger()));
    if (!runtime_) {
      throw std::runtime_error("failed to create TensorRT runtime");
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!engine_) {
      throw std::runtime_error("failed to deserialize TensorRT engine");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("failed to create TensorRT execution context");
    }

    discover_tensors();
    allocate_io();
  }

  void discover_tensors()
  {
    const int nb_tensors = engine_->getNbIOTensors();
    for (int i = 0; i < nb_tensors; ++i) {
      const char* name = engine_->getIOTensorName(i);
      if (name == nullptr) {
        continue;
      }
      const auto mode = engine_->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT && input_name_.empty()) {
        input_name_ = name;
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT && output_name_.empty()) {
        output_name_ = name;
      }
    }

    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT engine must expose at least one input and one output tensor");
    }

    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    output_type_ = engine_->getTensorDataType(output_name_.c_str());

    auto input_dims = engine_->getTensorShape(input_name_.c_str());
    if (input_dims.nbDims != 4) {
      throw std::runtime_error("expected YOLO input dims [N,C,H,W], got " + dims_to_string(input_dims));
    }

    if (input_dims.d[0] < 0 || input_dims.d[1] < 0 || input_dims.d[2] < 0 || input_dims.d[3] < 0) {
      input_dims.d[0] = input_dims.d[0] < 0 ? 1 : input_dims.d[0];
      input_dims.d[1] = input_dims.d[1] < 0 ? 3 : input_dims.d[1];
      input_dims.d[2] = input_dims.d[2] < 0 ? kDefaultNetworkSize : input_dims.d[2];
      input_dims.d[3] = input_dims.d[3] < 0 ? kDefaultNetworkSize : input_dims.d[3];
      if (!context_->setInputShape(input_name_.c_str(), input_dims)) {
        throw std::runtime_error("failed to set dynamic YOLO input shape " + dims_to_string(input_dims));
      }
    }

    input_dims_ = context_->getTensorShape(input_name_.c_str());
    input_h_ = static_cast<int>(input_dims_.d[2]);
    input_w_ = static_cast<int>(input_dims_.d[3]);
    if (input_dims_.d[0] != 1 || input_dims_.d[1] != 3 || input_h_ <= 0 || input_w_ <= 0) {
      throw std::runtime_error("expected YOLO input shape [1,3,H,W], got " + dims_to_string(input_dims_));
    }
  }

  void allocate_io()
  {
    input_elements_ = volume(input_dims_);
    const auto input_bytes = input_elements_ * data_type_size(input_type_);
    if (input_bytes == 0) {
      throw std::runtime_error("invalid TensorRT input allocation size");
    }
    input_buffer_.resize(input_bytes);
    if (!context_->setTensorAddress(input_name_.c_str(), input_buffer_.ptr)) {
      throw std::runtime_error("failed to bind TensorRT input tensor");
    }

    const int missing_shapes = context_->inferShapes(0, nullptr);
    if (missing_shapes < 0) {
      throw std::runtime_error("TensorRT shape inference failed");
    }

    output_dims_ = context_->getTensorShape(output_name_.c_str());
    output_elements_ = volume(output_dims_);
    const auto output_bytes = output_elements_ * data_type_size(output_type_);
    if (output_bytes == 0) {
      throw std::runtime_error("invalid TensorRT output shape " + dims_to_string(output_dims_));
    }

    output_buffer_.resize(output_bytes);
    output_host_bytes_.resize(output_bytes);
    output_float_.resize(output_elements_);
    if (!context_->setTensorAddress(output_name_.c_str(), output_buffer_.ptr)) {
      throw std::runtime_error("failed to bind TensorRT output tensor");
    }

    input_float_.resize(input_elements_);
  }

  void upload_input()
  {
    if (input_type_ == nvinfer1::DataType::kFLOAT) {
      check_cuda(
        cudaMemcpyAsync(
          input_buffer_.ptr,
          input_float_.data(),
          input_float_.size() * sizeof(float),
          cudaMemcpyHostToDevice,
          stream_.stream),
        "cudaMemcpyAsync input fp32");
      return;
    }

    if (input_type_ == nvinfer1::DataType::kHALF) {
      input_half_.resize(input_float_.size());
      std::transform(input_float_.begin(), input_float_.end(), input_half_.begin(), [](float value) {
        return __float2half(value);
      });
      check_cuda(
        cudaMemcpyAsync(
          input_buffer_.ptr,
          input_half_.data(),
          input_half_.size() * sizeof(__half),
          cudaMemcpyHostToDevice,
          stream_.stream),
        "cudaMemcpyAsync input fp16");
      return;
    }

    throw std::runtime_error("unsupported TensorRT input data type for YOLO backend");
  }

  const std::vector<float>& output_as_float()
  {
    if (output_type_ == nvinfer1::DataType::kFLOAT) {
      const auto* raw = reinterpret_cast<const float*>(output_host_bytes_.data());
      std::copy(raw, raw + output_elements_, output_float_.begin());
      return output_float_;
    }

    if (output_type_ == nvinfer1::DataType::kHALF) {
      const auto* raw = reinterpret_cast<const __half*>(output_host_bytes_.data());
      for (std::size_t i = 0; i < output_elements_; ++i) {
        output_float_[i] = __half2float(raw[i]);
      }
      return output_float_;
    }

    throw std::runtime_error("unsupported TensorRT output data type for YOLO backend");
  }

  std::vector<Candidate> decode(const Letterbox& letterbox, float image_w, float image_h)
  {
    const auto& out = output_as_float();
    const int nb_dims = output_dims_.nbDims;
    if (nb_dims < 2 || nb_dims > 3) {
      throw std::runtime_error("unsupported YOLO output shape " + dims_to_string(output_dims_));
    }

    if (nb_dims == 2 && output_dims_.d[1] == 6) {
      return decode_nms_like(out, output_dims_.d[0], letterbox, image_w, image_h);
    }

    int rows = 0;
    int attrs = 0;
    bool transposed = false;
    if (nb_dims == 3) {
      const int a = static_cast<int>(output_dims_.d[1]);
      const int b = static_cast<int>(output_dims_.d[2]);
      const int expected = static_cast<int>(labels_.size()) + 4;
      if (a == expected || (a <= 512 && b > a)) {
        attrs = a;
        rows = b;
        transposed = true;
      } else {
        rows = a;
        attrs = b;
      }
    } else {
      rows = static_cast<int>(output_dims_.d[0]);
      attrs = static_cast<int>(output_dims_.d[1]);
    }

    if (attrs == 6) {
      return decode_nms_like(out, rows, letterbox, image_w, image_h);
    }
    if (attrs < 5) {
      throw std::runtime_error("YOLO output attributes too small: " + std::to_string(attrs));
    }

    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(rows / 16));
    const int class_count = attrs - 4;

    for (int row = 0; row < rows; ++row) {
      auto at = [&](int attr) -> float {
        if (transposed) {
          return out[static_cast<std::size_t>(attr) * static_cast<std::size_t>(rows) + row];
        }
        return out[static_cast<std::size_t>(row) * static_cast<std::size_t>(attrs) + attr];
      };

      int best_class = 0;
      float best_score = at(4);
      for (int c = 1; c < class_count; ++c) {
        const float score = at(4 + c);
        if (score > best_score) {
          best_score = score;
          best_class = c;
        }
      }
      if (best_score < config_.confidence_threshold) {
        continue;
      }

      const float cx = at(0);
      const float cy = at(1);
      const float w = at(2);
      const float h = at(3);
      candidates.push_back(from_model_box(cx - w * 0.5f, cy - h * 0.5f, cx + w * 0.5f, cy + h * 0.5f,
                                          best_score, best_class, letterbox, image_w, image_h));
    }

    return candidates;
  }

  std::vector<Candidate> decode_nms_like(
    const std::vector<float>& out,
    int rows,
    const Letterbox& letterbox,
    float image_w,
    float image_h)
  {
    std::vector<Candidate> candidates;
    for (int row = 0; row < rows; ++row) {
      const auto base = static_cast<std::size_t>(row) * 6;
      const float score = out[base + 4];
      if (score < config_.confidence_threshold) {
        continue;
      }
      candidates.push_back(from_model_box(
        out[base + 0],
        out[base + 1],
        out[base + 2],
        out[base + 3],
        score,
        static_cast<int>(std::round(out[base + 5])),
        letterbox,
        image_w,
        image_h));
    }
    return candidates;
  }

  Candidate from_model_box(
    float x1,
    float y1,
    float x2,
    float y2,
    float score,
    int class_index,
    const Letterbox& letterbox,
    float image_w,
    float image_h) const
  {
    x1 = (x1 - letterbox.pad_x) / letterbox.scale;
    y1 = (y1 - letterbox.pad_y) / letterbox.scale;
    x2 = (x2 - letterbox.pad_x) / letterbox.scale;
    y2 = (y2 - letterbox.pad_y) / letterbox.scale;

    Candidate candidate;
    candidate.class_index = class_index;
    candidate.score = score;
    candidate.x1 = std::clamp(x1, 0.0f, image_w - 1.0f);
    candidate.y1 = std::clamp(y1, 0.0f, image_h - 1.0f);
    candidate.x2 = std::clamp(x2, 0.0f, image_w - 1.0f);
    candidate.y2 = std::clamp(y2, 0.0f, image_h - 1.0f);
    return candidate;
  }

  std::vector<amr_interfaces::msg::Detection2D> to_messages(const std::vector<Candidate>& candidates) const
  {
    std::vector<amr_interfaces::msg::Detection2D> detections;
    detections.reserve(candidates.size());
    for (const auto& candidate : candidates) {
      if (candidate.x2 <= candidate.x1 || candidate.y2 <= candidate.y1) {
        continue;
      }

      amr_interfaces::msg::Detection2D detection;
      detection.detector_id = config_.model_name;
      detection.class_index = candidate.class_index;
      detection.class_id = candidate.class_index >= 0 &&
          static_cast<std::size_t>(candidate.class_index) < labels_.size() ?
        labels_[static_cast<std::size_t>(candidate.class_index)] :
        std::to_string(candidate.class_index);
      detection.score = candidate.score;
      detection.center_x = (candidate.x1 + candidate.x2) * 0.5f;
      detection.center_y = (candidate.y1 + candidate.y2) * 0.5f;
      detection.size_x = candidate.x2 - candidate.x1;
      detection.size_y = candidate.y2 - candidate.y1;
      detections.push_back(std::move(detection));
    }
    return detections;
  }

  DetectorConfig config_;
  std::vector<std::string> labels_;
  bool ready_{false};
  std::string status_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  CudaStream stream_;
  DeviceBuffer input_buffer_;
  DeviceBuffer output_buffer_;

  std::string input_name_;
  std::string output_name_;
  nvinfer1::DataType input_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  int input_w_{kDefaultNetworkSize};
  int input_h_{kDefaultNetworkSize};
  std::size_t input_elements_{0};
  std::size_t output_elements_{0};

  std::vector<float> input_float_;
  std::vector<__half> input_half_;
  std::vector<std::byte> output_host_bytes_;
  std::vector<float> output_float_;
};

std::string normalized(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace

std::vector<std::string> load_labels(const std::string& path)
{
  if (path.empty()) {
    return {};
  }

  std::ifstream input(path);
  if (!input) {
    return {};
  }

  std::vector<std::string> labels;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      labels.push_back(line);
    }
  }
  return labels;
}

std::unique_ptr<DetectorBackend> make_detector_backend(const DetectorConfig& config)
{
  const auto backend = normalized(config.backend);

  if (backend == "debug" || backend == "null") {
    return std::make_unique<DebugDetectorBackend>(config);
  }

  if (backend == "yolo_tensorrt") {
    return std::make_unique<TensorRtYoloBackend>(config, load_labels(config.labels_path));
  }

  if (backend == "rf_detr_tensorrt" || backend == "rtdetr_tensorrt") {
    DetectorConfig fallback = config;
    fallback.backend = "debug";
    fallback.model_name = config.model_name + "_not_implemented";
    fallback.publish_debug_detections = false;
    return std::make_unique<DebugDetectorBackend>(fallback);
  }

  DetectorConfig fallback = config;
  fallback.backend = "debug";
  fallback.model_name = "unsupported_backend_debug_fallback";
  fallback.publish_debug_detections = false;
  return std::make_unique<DebugDetectorBackend>(fallback);
}

}  // namespace perception_inference
