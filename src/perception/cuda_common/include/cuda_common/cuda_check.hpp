#pragma once

#include <cuda_runtime.h>

#include <stdexcept>
#include <string>

namespace cuda_common
{

class CudaError : public std::runtime_error
{
public:
  CudaError(cudaError_t error, const char* expression, const char* file, int line)
  : std::runtime_error(build_message(error, expression, file, line)), error_(error)
  {
  }

  cudaError_t error() const noexcept { return error_; }

private:
  static std::string build_message(cudaError_t error, const char* expression, const char* file, int line)
  {
    return std::string("CUDA call failed: ") + expression + " at " + file + ":" +
           std::to_string(line) + " -> " + cudaGetErrorString(error);
  }

  cudaError_t error_;
};

inline void throw_if_failed(cudaError_t error, const char* expression, const char* file, int line)
{
  if (error != cudaSuccess) {
    throw CudaError(error, expression, file, line);
  }
}

}  // namespace cuda_common

#define CUDA_COMMON_CHECK(expr) \
  ::cuda_common::throw_if_failed((expr), #expr, __FILE__, __LINE__)

