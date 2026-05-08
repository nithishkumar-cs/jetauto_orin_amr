#pragma once

#include "cuda_common/cuda_check.hpp"

#include <cuda_runtime.h>

namespace cuda_common
{

class CudaEventTimer
{
public:
  CudaEventTimer()
  {
    CUDA_COMMON_CHECK(cudaEventCreateWithFlags(&start_, cudaEventDefault));
    CUDA_COMMON_CHECK(cudaEventCreateWithFlags(&stop_, cudaEventDefault));
  }

  ~CudaEventTimer()
  {
    if (start_ != nullptr) {
      cudaEventDestroy(start_);
    }
    if (stop_ != nullptr) {
      cudaEventDestroy(stop_);
    }
  }

  CudaEventTimer(const CudaEventTimer&) = delete;
  CudaEventTimer& operator=(const CudaEventTimer&) = delete;

  void start(cudaStream_t stream)
  {
    CUDA_COMMON_CHECK(cudaEventRecord(start_, stream));
  }

  void stop(cudaStream_t stream)
  {
    CUDA_COMMON_CHECK(cudaEventRecord(stop_, stream));
  }

  float elapsed_ms()
  {
    CUDA_COMMON_CHECK(cudaEventSynchronize(stop_));
    float elapsed = 0.0f;
    CUDA_COMMON_CHECK(cudaEventElapsedTime(&elapsed, start_, stop_));
    return elapsed;
  }

private:
  cudaEvent_t start_{nullptr};
  cudaEvent_t stop_{nullptr};
};

}  // namespace cuda_common

