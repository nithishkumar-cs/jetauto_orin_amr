#ifndef PERCEPTION_DETECTOR__YOLO_PREPROCESSOR_HPP_
#define PERCEPTION_DETECTOR__YOLO_PREPROCESSOR_HPP_

#include <cuda_runtime_api.h>

#include <cstddef>

#include "perception_detector/detector_backend.hpp"
#include "postprocessing.hpp"

namespace perception_detector
{

enum class YoloTensorType {
  FLOAT32,
  FLOAT16,
};

struct YoloPreprocessDestination
{
  void * data{nullptr};
  std::size_t size_bytes{0U};
  YoloTensorType type{YoloTensorType::FLOAT32};
  cudaStream_t stream{nullptr};
};

class YoloPreprocessor
{
public:
  virtual ~YoloPreprocessor() = default;

  virtual const char * name() const = 0;
  virtual LetterboxTransform preprocess(
    const ImageView & image, std::size_t input_width, std::size_t input_height,
    const YoloPreprocessDestination & destination) = 0;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_PREPROCESSOR_HPP_
