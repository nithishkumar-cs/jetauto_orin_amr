#include "runtime/tensorrt/session.hpp"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace perception_detector::tensorrt
{
namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING && message != nullptr) {
      std::scoped_lock lock(mutex_);
      last_warning_ = message;
    }
  }

  std::string last_warning() const
  {
    std::scoped_lock lock(mutex_);
    return last_warning_;
  }

private:
  mutable std::mutex mutex_;
  std::string last_warning_;
};

void check_cuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

std::vector<char> read_engine_file(const std::string & path)
{
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("TensorRT engine does not exist: " + path);
  }

  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) {
    throw std::runtime_error("failed to open TensorRT engine: " + path);
  }
  const auto end = input.tellg();
  if (end <= 0) {
    throw std::runtime_error("TensorRT engine is empty: " + path);
  }
  const auto size = static_cast<std::size_t>(end);
  std::vector<char> bytes(size);
  input.seekg(0, std::ios::beg);
  if (!input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()))) {
    throw std::runtime_error("failed to read TensorRT engine: " + path);
  }
  return bytes;
}

std::string shape_string(const nvinfer1::Dims & shape)
{
  std::ostringstream output;
  output << '[';
  for (std::int32_t index = 0; index < shape.nbDims; ++index) {
    if (index != 0) {
      output << ',';
    }
    output << shape.d[index];
  }
  output << ']';
  return output.str();
}

std::vector<std::int64_t> shape_vector(const nvinfer1::Dims & shape)
{
  std::vector<std::int64_t> result;
  if (shape.nbDims > 0) {
    result.reserve(static_cast<std::size_t>(shape.nbDims));
  }
  for (std::int32_t index = 0; index < shape.nbDims; ++index) {
    result.push_back(static_cast<std::int64_t>(shape.d[index]));
  }
  return result;
}

std::size_t tensor_volume(const nvinfer1::Dims & shape)
{
  if (shape.nbDims <= 0) {
    throw std::runtime_error("TensorRT tensor shape is empty");
  }

  std::size_t count = 1U;
  for (std::int32_t index = 0; index < shape.nbDims; ++index) {
    if (shape.d[index] <= 0) {
      throw std::runtime_error("TensorRT tensor has unresolved shape " + shape_string(shape));
    }
    const auto dimension = static_cast<std::size_t>(shape.d[index]);
    if (count > std::numeric_limits<std::size_t>::max() / dimension) {
      throw std::overflow_error("TensorRT tensor shape is too large");
    }
    count *= dimension;
  }
  return count;
}

TensorType tensor_type(nvinfer1::DataType type)
{
  if (type == nvinfer1::DataType::kFLOAT) {
    return TensorType::FLOAT32;
  }
  if (type == nvinfer1::DataType::kHALF) {
    return TensorType::FLOAT16;
  }
  throw std::runtime_error("TensorRT session supports only float32 and float16 tensors");
}

std::size_t tensor_element_size(TensorType type)
{
  return type == TensorType::FLOAT32 ? sizeof(float) : sizeof(__half);
}

class DeviceBuffer
{
public:
  explicit DeviceBuffer(std::size_t size_bytes) : size_bytes_(size_bytes)
  {
    if (size_bytes == 0U) {
      throw std::invalid_argument("cannot allocate an empty CUDA buffer");
    }
    check_cuda(cudaMalloc(&data_, size_bytes), "cudaMalloc TensorRT tensor");
  }

  ~DeviceBuffer()
  {
    if (data_ != nullptr) {
      cudaFree(data_);
    }
  }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;

  void * data() const
  {
    return data_;
  }

  std::size_t size_bytes() const
  {
    return size_bytes_;
  }

private:
  void * data_{nullptr};
  std::size_t size_bytes_{0U};
};

struct BufferEntry
{
  std::string name;
  TensorType type{TensorType::FLOAT32};
  std::unique_ptr<DeviceBuffer> storage;
};

}  // namespace

class Session::Impl
{
public:
  explicit Impl(const std::string & engine_path)
  {
    const auto engine_bytes = read_engine_file(engine_path);
    if (!initLibNvInferPlugins(&logger_, "")) {
      throw std::runtime_error("failed to initialize TensorRT plugins");
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("failed to create the TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(engine_bytes.data(), engine_bytes.size()));
    if (!engine_) {
      const auto warning = logger_.last_warning();
      throw std::runtime_error(
        "failed to deserialize TensorRT engine" +
        (warning.empty() ? std::string{} : ": " + warning));
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("failed to create the TensorRT execution context");
    }

    discover_tensors();
    check_cuda(cudaStreamCreate(&stream_), "cudaStreamCreate TensorRT session");
  }

  ~Impl()
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  const std::vector<std::string> & input_names() const
  {
    return input_names_;
  }

  const std::vector<std::string> & output_names() const
  {
    return output_names_;
  }

  std::vector<std::int64_t> tensor_shape(const std::string & name) const
  {
    require_known_tensor(name);
    return shape_vector(context_->getTensorShape(name.c_str()));
  }

  void set_input_shape(const std::string & name, const std::vector<std::int64_t> & shape)
  {
    require_not_prepared("set an input shape");
    require_tensor_mode(name, nvinfer1::TensorIOMode::kINPUT);

    auto dimensions = engine_->getTensorShape(name.c_str());
    if (dimensions.nbDims <= 0 || shape.size() != static_cast<std::size_t>(dimensions.nbDims)) {
      throw std::invalid_argument("configured TensorRT input shape has the wrong rank");
    }
    for (std::int32_t index = 0; index < dimensions.nbDims; ++index) {
      const auto value = shape[static_cast<std::size_t>(index)];
      if (value <= 0 || value > std::numeric_limits<std::int32_t>::max()) {
        throw std::invalid_argument(
          "configured TensorRT input shape contains an invalid dimension");
      }
      dimensions.d[index] = static_cast<std::int32_t>(value);
    }
    if (!context_->setInputShape(name.c_str(), dimensions)) {
      throw std::runtime_error(
        "failed to select TensorRT input shape " + shape_string(dimensions) + " for '" + name +
        "'");
    }
  }

  void prepare()
  {
    if (prepared_) {
      return;
    }

    for (const auto & name : input_names_) {
      allocate_and_bind(name);
    }
    const auto insufficient_inputs = context_->inferShapes(0, nullptr);
    if (insufficient_inputs != 0) {
      throw std::runtime_error("TensorRT could not resolve all tensor shapes");
    }
    for (const auto & name : output_names_) {
      allocate_and_bind(name);
    }
    prepared_ = true;
  }

  TensorBufferView buffer(const std::string & name) const
  {
    require_prepared("access a tensor buffer");
    const auto & entry = find_buffer(name);
    return TensorBufferView{entry.storage->data(), entry.storage->size_bytes(), entry.type};
  }

  cudaStream_t stream() const
  {
    return stream_;
  }

  void enqueue()
  {
    require_prepared("enqueue inference");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
  }

  std::vector<float> download_output(const std::string & name)
  {
    require_prepared("download a tensor output");
    require_tensor_mode(name, nvinfer1::TensorIOMode::kOUTPUT);
    const auto & entry = find_buffer(name);
    const auto element_count = entry.storage->size_bytes() / tensor_element_size(entry.type);
    std::vector<float> output(element_count);

    if (entry.type == TensorType::FLOAT32) {
      check_cuda(
        cudaMemcpyAsync(
          output.data(), entry.storage->data(), entry.storage->size_bytes(), cudaMemcpyDeviceToHost,
          stream_),
        "cudaMemcpyAsync TensorRT output");
      check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize TensorRT output");
      return output;
    }

    std::vector<__half> half_output(element_count);
    check_cuda(
      cudaMemcpyAsync(
        half_output.data(), entry.storage->data(), entry.storage->size_bytes(),
        cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync TensorRT FP16 output");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize TensorRT output");
    std::transform(half_output.begin(), half_output.end(), output.begin(), [](__half value) {
      return __half2float(value);
    });
    return output;
  }

private:
  void discover_tensors()
  {
    for (std::int32_t index = 0; index < engine_->getNbIOTensors(); ++index) {
      const auto * name = engine_->getIOTensorName(index);
      if (name == nullptr) {
        throw std::runtime_error("TensorRT engine contains an unnamed I/O tensor");
      }
      tensor_type(engine_->getTensorDataType(name));
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_names_.emplace_back(name);
      } else {
        output_names_.emplace_back(name);
      }
    }
    if (input_names_.empty() || output_names_.empty()) {
      throw std::runtime_error("TensorRT engine must contain at least one input and one output");
    }
  }

  void allocate_and_bind(const std::string & name)
  {
    const auto dimensions = context_->getTensorShape(name.c_str());
    const auto type = tensor_type(engine_->getTensorDataType(name.c_str()));
    const auto element_count = tensor_volume(dimensions);
    const auto element_size = tensor_element_size(type);
    if (element_count > std::numeric_limits<std::size_t>::max() / element_size) {
      throw std::overflow_error("TensorRT tensor buffer is too large");
    }
    const auto size_bytes = element_count * element_size;
    auto storage = std::make_unique<DeviceBuffer>(size_bytes);
    if (!context_->setTensorAddress(name.c_str(), storage->data())) {
      throw std::runtime_error("failed to bind TensorRT tensor '" + name + "'");
    }
    buffers_.push_back(BufferEntry{name, type, std::move(storage)});
  }

  const BufferEntry & find_buffer(const std::string & name) const
  {
    const auto found = std::find_if(
      buffers_.begin(), buffers_.end(), [&](const auto & entry) { return entry.name == name; });
    if (found == buffers_.end()) {
      throw std::invalid_argument("unknown TensorRT tensor buffer '" + name + "'");
    }
    return *found;
  }

  void require_known_tensor(const std::string & name) const
  {
    if (engine_->getTensorIOMode(name.c_str()) == nvinfer1::TensorIOMode::kNONE) {
      throw std::invalid_argument("unknown TensorRT tensor '" + name + "'");
    }
  }

  void require_tensor_mode(const std::string & name, nvinfer1::TensorIOMode expected) const
  {
    require_known_tensor(name);
    if (engine_->getTensorIOMode(name.c_str()) != expected) {
      throw std::invalid_argument("TensorRT tensor '" + name + "' has the wrong I/O mode");
    }
  }

  void require_prepared(const std::string & operation) const
  {
    if (!prepared_) {
      throw std::logic_error("cannot " + operation + " before preparing the TensorRT session");
    }
  }

  void require_not_prepared(const std::string & operation) const
  {
    if (prepared_) {
      throw std::logic_error("cannot " + operation + " after preparing the TensorRT session");
    }
  }

  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<BufferEntry> buffers_;
  cudaStream_t stream_{nullptr};
  bool prepared_{false};
};

Session::Session(const std::string & engine_path) : impl_(std::make_unique<Impl>(engine_path)) {}
Session::~Session() = default;

const std::vector<std::string> & Session::input_names() const
{
  return impl_->input_names();
}

const std::vector<std::string> & Session::output_names() const
{
  return impl_->output_names();
}

std::vector<std::int64_t> Session::tensor_shape(const std::string & name) const
{
  return impl_->tensor_shape(name);
}

void Session::set_input_shape(const std::string & name, const std::vector<std::int64_t> & shape)
{
  impl_->set_input_shape(name, shape);
}

void Session::prepare()
{
  impl_->prepare();
}

TensorBufferView Session::buffer(const std::string & name) const
{
  return impl_->buffer(name);
}

cudaStream_t Session::stream() const
{
  return impl_->stream();
}

void Session::enqueue()
{
  impl_->enqueue();
}

std::vector<float> Session::download_output(const std::string & name)
{
  return impl_->download_output(name);
}

}  // namespace perception_detector::tensorrt
