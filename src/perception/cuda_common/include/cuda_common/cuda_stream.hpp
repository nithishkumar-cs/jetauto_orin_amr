#pragma once

#include "cuda_common/cuda_check.hpp"

#include <cuda_runtime.h>

namespace cuda_common
{

class CudaStream
{
public:
  explicit CudaStream(unsigned int flags = cudaStreamNonBlocking)
  {
    CUDA_COMMON_CHECK(cudaStreamCreateWithFlags(&stream_, flags));
  }

  ~CudaStream()
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  CudaStream(const CudaStream&) = delete;
  CudaStream& operator=(const CudaStream&) = delete;

  CudaStream(CudaStream&& other) noexcept
  : stream_(other.stream_)
  {
    other.stream_ = nullptr;
  }

  CudaStream& operator=(CudaStream&& other) noexcept
  {
    if (this != &other) {
      if (stream_ != nullptr) {
        cudaStreamDestroy(stream_);
      }
      stream_ = other.stream_;
      other.stream_ = nullptr;
    }
    return *this;
  }

  cudaStream_t get() const noexcept { return stream_; }

  void synchronize() const
  {
    CUDA_COMMON_CHECK(cudaStreamSynchronize(stream_));
  }

private:
  cudaStream_t stream_{nullptr};
};

}  // namespace cuda_common

