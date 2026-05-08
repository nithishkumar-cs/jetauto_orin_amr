#include "cuda_common/device_info.hpp"

#include "cuda_common/cuda_check.hpp"

#include <cuda_runtime.h>

#include <sstream>

namespace cuda_common
{

std::string active_device_summary()
{
  int device = 0;
  CUDA_COMMON_CHECK(cudaGetDevice(&device));

  cudaDeviceProp prop{};
  CUDA_COMMON_CHECK(cudaGetDeviceProperties(&prop, device));

  std::ostringstream out;
  out << "cuda_device=" << device
      << " name=\"" << prop.name << "\""
      << " sm=" << prop.major << "." << prop.minor
      << " global_mem_mb=" << (prop.totalGlobalMem / (1024 * 1024))
      << " multiprocessors=" << prop.multiProcessorCount;
  return out.str();
}

}  // namespace cuda_common

