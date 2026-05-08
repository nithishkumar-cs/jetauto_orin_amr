#pragma once

#include "cuda_common/cuda_check.hpp"

#include <cuda_runtime.h>

#include <cstddef>
#include <utility>

namespace cuda_common
{

template <typename T>
class PinnedHostBuffer
{
public:
  PinnedHostBuffer() = default;

  explicit PinnedHostBuffer(std::size_t count)
  {
    reserve(count);
    size_ = count;
  }

  ~PinnedHostBuffer()
  {
    release();
  }

  PinnedHostBuffer(const PinnedHostBuffer&) = delete;
  PinnedHostBuffer& operator=(const PinnedHostBuffer&) = delete;

  PinnedHostBuffer(PinnedHostBuffer&& other) noexcept
  {
    move_from(std::move(other));
  }

  PinnedHostBuffer& operator=(PinnedHostBuffer&& other) noexcept
  {
    if (this != &other) {
      release();
      move_from(std::move(other));
    }
    return *this;
  }

  void reserve(std::size_t count)
  {
    if (count <= capacity_) {
      return;
    }

    T* next = nullptr;
    CUDA_COMMON_CHECK(cudaMallocHost(reinterpret_cast<void**>(&next), count * sizeof(T)));
    release();
    data_ = next;
    capacity_ = count;
    size_ = 0;
  }

  void resize(std::size_t count)
  {
    reserve(count);
    size_ = count;
  }

  void release() noexcept
  {
    if (data_ != nullptr) {
      cudaFreeHost(data_);
      data_ = nullptr;
    }
    size_ = 0;
    capacity_ = 0;
  }

  T* data() noexcept { return data_; }
  const T* data() const noexcept { return data_; }
  std::size_t size() const noexcept { return size_; }
  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t bytes() const noexcept { return size_ * sizeof(T); }

private:
  void move_from(PinnedHostBuffer&& other) noexcept
  {
    data_ = other.data_;
    size_ = other.size_;
    capacity_ = other.capacity_;
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
  }

  T* data_{nullptr};
  std::size_t size_{0};
  std::size_t capacity_{0};
};

}  // namespace cuda_common

