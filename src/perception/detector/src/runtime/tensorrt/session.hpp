#ifndef PERCEPTION_DETECTOR__RUNTIME__TENSORRT__SESSION_HPP_
#define PERCEPTION_DETECTOR__RUNTIME__TENSORRT__SESSION_HPP_

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace perception_detector::tensorrt
{

enum class TensorType {
  FLOAT32,
  FLOAT16,
};

struct TensorBufferView
{
  void * data{nullptr};
  std::size_t size_bytes{0U};
  TensorType type{TensorType::FLOAT32};
};

// Owns one deserialized TensorRT engine, execution context, CUDA stream, and
// bound device buffers. Model-specific backends configure shapes and consume
// the buffers without depending on TensorRT's C++ types.
class Session
{
public:
  explicit Session(const std::string & engine_path);
  ~Session();

  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;

  const std::vector<std::string> & input_names() const;
  const std::vector<std::string> & output_names() const;
  std::vector<std::int64_t> tensor_shape(const std::string & name) const;

  void set_input_shape(const std::string & name, const std::vector<std::int64_t> & shape);
  void prepare();

  TensorBufferView buffer(const std::string & name) const;
  cudaStream_t stream() const;
  void enqueue();
  std::vector<float> download_output(const std::string & name);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception_detector::tensorrt

#endif  // PERCEPTION_DETECTOR__RUNTIME__TENSORRT__SESSION_HPP_
