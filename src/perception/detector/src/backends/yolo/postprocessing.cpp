#include "postprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace perception_detector
{
namespace
{

struct Candidate
{
  std::size_t class_index{0U};
  double score{0.0};
  double left{0.0};
  double top{0.0};
  double right{0.0};
  double bottom{0.0};
};

std::size_t checked_element_count(const std::vector<std::int64_t> & shape)
{
  if (shape.empty()) {
    throw std::invalid_argument("YOLO output shape is empty");
  }

  std::size_t count = 1U;
  for (const auto dimension : shape) {
    if (dimension <= 0) {
      throw std::invalid_argument("YOLO output shape contains a non-positive dimension");
    }
    const auto value = static_cast<std::size_t>(dimension);
    if (count > std::numeric_limits<std::size_t>::max() / value) {
      throw std::overflow_error("YOLO output shape is too large");
    }
    count *= value;
  }
  return count;
}

double intersection_over_union(const Candidate & first, const Candidate & second)
{
  const auto intersection_left = std::max(first.left, second.left);
  const auto intersection_top = std::max(first.top, second.top);
  const auto intersection_right = std::min(first.right, second.right);
  const auto intersection_bottom = std::min(first.bottom, second.bottom);
  const auto intersection_width = std::max(0.0, intersection_right - intersection_left);
  const auto intersection_height = std::max(0.0, intersection_bottom - intersection_top);
  const auto intersection_area = intersection_width * intersection_height;
  const auto first_area =
    std::max(0.0, first.right - first.left) * std::max(0.0, first.bottom - first.top);
  const auto second_area =
    std::max(0.0, second.right - second.left) * std::max(0.0, second.bottom - second.top);
  const auto union_area = first_area + second_area - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

Candidate map_to_image(
  double left, double top, double right, double bottom, double score, std::size_t class_index,
  const LetterboxTransform & transform, std::size_t image_width, std::size_t image_height)
{
  if (transform.scale_x <= 0.0 || transform.scale_y <= 0.0) {
    throw std::invalid_argument("letterbox transform has a non-positive scale");
  }

  const auto max_x = static_cast<double>(image_width);
  const auto max_y = static_cast<double>(image_height);
  Candidate candidate;
  candidate.class_index = class_index;
  candidate.score = score;
  candidate.left = std::clamp((left - transform.pad_left) / transform.scale_x, 0.0, max_x);
  candidate.top = std::clamp((top - transform.pad_top) / transform.scale_y, 0.0, max_y);
  candidate.right = std::clamp((right - transform.pad_left) / transform.scale_x, 0.0, max_x);
  candidate.bottom = std::clamp((bottom - transform.pad_top) / transform.scale_y, 0.0, max_y);
  return candidate;
}

std::vector<Candidate> decode_raw(
  const std::vector<float> & output, const std::vector<std::int64_t> & shape,
  const LetterboxTransform & transform, std::size_t image_width, std::size_t image_height,
  std::size_t label_count, double confidence_threshold)
{
  if (shape.size() != 2U && shape.size() != 3U) {
    throw std::invalid_argument("raw YOLO output must have two or three dimensions");
  }
  if (shape.size() == 3U && shape[0] != 1) {
    throw std::invalid_argument("raw YOLO output must use batch size one");
  }

  const auto first = static_cast<std::size_t>(shape[shape.size() - 2U]);
  const auto second = static_cast<std::size_t>(shape[shape.size() - 1U]);
  const auto expected_attributes = label_count == 0U ? 0U : label_count + 4U;

  bool transposed = false;
  std::size_t attributes = 0U;
  std::size_t predictions = 0U;
  if (expected_attributes != 0U && first == expected_attributes) {
    attributes = first;
    predictions = second;
    transposed = true;
  } else if (expected_attributes != 0U && second == expected_attributes) {
    predictions = first;
    attributes = second;
  } else if (first >= 5U && first < second) {
    attributes = first;
    predictions = second;
    transposed = true;
  } else if (second >= 5U) {
    predictions = first;
    attributes = second;
  } else {
    throw std::invalid_argument("cannot identify the attribute axis in raw YOLO output");
  }

  const auto class_count = attributes - 4U;
  if (label_count != 0U && class_count != label_count) {
    throw std::invalid_argument("YOLO output class count does not match the labels file");
  }

  std::vector<Candidate> candidates;
  for (std::size_t prediction = 0U; prediction < predictions; ++prediction) {
    const auto at = [&](std::size_t attribute) {
      const auto index =
        transposed ? attribute * predictions + prediction : prediction * attributes + attribute;
      return static_cast<double>(output.at(index));
    };

    std::size_t best_class = 0U;
    auto best_score = at(4U);
    for (std::size_t class_index = 1U; class_index < class_count; ++class_index) {
      const auto score = at(4U + class_index);
      if (score > best_score) {
        best_score = score;
        best_class = class_index;
      }
    }
    if (!std::isfinite(best_score) || best_score < confidence_threshold || best_score > 1.0) {
      continue;
    }

    const auto center_x = at(0U);
    const auto center_y = at(1U);
    const auto width = at(2U);
    const auto height = at(3U);
    if (
      !std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(width) ||
      !std::isfinite(height) || width <= 0.0 || height <= 0.0) {
      continue;
    }
    candidates.push_back(map_to_image(
      center_x - width / 2.0, center_y - height / 2.0, center_x + width / 2.0,
      center_y + height / 2.0, best_score, best_class, transform, image_width, image_height));
  }
  return candidates;
}

std::vector<Candidate> decode_end_to_end(
  const std::vector<float> & output, const std::vector<std::int64_t> & shape,
  const LetterboxTransform & transform, std::size_t image_width, std::size_t image_height,
  double confidence_threshold)
{
  if (shape.size() != 2U && shape.size() != 3U) {
    throw std::invalid_argument("end-to-end YOLO output must have two or three dimensions");
  }
  if (shape.size() == 3U && shape[0] != 1) {
    throw std::invalid_argument("end-to-end YOLO output must use batch size one");
  }
  if (shape.back() != 6) {
    throw std::invalid_argument("end-to-end YOLO output rows must contain six values");
  }

  const auto rows = static_cast<std::size_t>(shape[shape.size() - 2U]);
  std::vector<Candidate> candidates;
  for (std::size_t row = 0U; row < rows; ++row) {
    const auto offset = row * 6U;
    const auto left = static_cast<double>(output.at(offset));
    const auto top = static_cast<double>(output.at(offset + 1U));
    const auto right = static_cast<double>(output.at(offset + 2U));
    const auto bottom = static_cast<double>(output.at(offset + 3U));
    const auto score = static_cast<double>(output.at(offset + 4U));
    const auto class_value = static_cast<double>(output.at(offset + 5U));
    const auto rounded_class = std::round(class_value);
    if (
      !std::isfinite(left) || !std::isfinite(top) || !std::isfinite(right) ||
      !std::isfinite(bottom) || !std::isfinite(score) || score < confidence_threshold ||
      score > 1.0 || !std::isfinite(class_value) || class_value < 0.0 ||
      class_value > static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
      std::abs(class_value - rounded_class) > 1e-6) {
      continue;
    }
    candidates.push_back(map_to_image(
      left, top, right, bottom, score, static_cast<std::size_t>(rounded_class), transform,
      image_width, image_height));
  }
  return candidates;
}

std::vector<Candidate> apply_nms(
  std::vector<Candidate> candidates, double iou_threshold, std::size_t max_detections)
{
  std::sort(candidates.begin(), candidates.end(), [](const auto & first, const auto & second) {
    return first.score > second.score;
  });

  std::vector<Candidate> kept;
  std::vector<bool> removed(candidates.size(), false);
  for (std::size_t index = 0U; index < candidates.size() && kept.size() < max_detections; ++index) {
    if (removed[index]) {
      continue;
    }
    if (
      candidates[index].right <= candidates[index].left ||
      candidates[index].bottom <= candidates[index].top) {
      continue;
    }
    kept.push_back(candidates[index]);
    for (std::size_t other = index + 1U; other < candidates.size(); ++other) {
      if (
        !removed[other] && candidates[index].class_index == candidates[other].class_index &&
        intersection_over_union(candidates[index], candidates[other]) > iou_threshold) {
        removed[other] = true;
      }
    }
  }
  return kept;
}

std::vector<Candidate> take_highest_scoring(
  std::vector<Candidate> candidates, std::size_t max_detections)
{
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const auto & first, const auto & second) { return first.score > second.score; });
  if (candidates.size() > max_detections) {
    candidates.resize(max_detections);
  }
  return candidates;
}

bool valid_final_candidate(
  const Candidate & candidate, std::size_t image_width, std::size_t image_height)
{
  const auto max_x = static_cast<double>(image_width);
  const auto max_y = static_cast<double>(image_height);
  return std::isfinite(candidate.left) && std::isfinite(candidate.top) &&
         std::isfinite(candidate.right) && std::isfinite(candidate.bottom) &&
         std::isfinite(candidate.score) && candidate.left >= 0.0 && candidate.top >= 0.0 &&
         candidate.right <= max_x && candidate.bottom <= max_y &&
         candidate.right > candidate.left && candidate.bottom > candidate.top &&
         candidate.score >= 0.0 && candidate.score <= 1.0;
}

}  // namespace

std::vector<Detection> postprocess_yolo(
  const std::vector<float> & output, const std::vector<std::int64_t> & output_shape,
  const LetterboxTransform & transform, std::size_t image_width, std::size_t image_height,
  const std::vector<std::string> & labels, const YoloPostprocessConfig & config)
{
  if (checked_element_count(output_shape) != output.size()) {
    throw std::invalid_argument("YOLO output buffer size does not match its shape");
  }
  if (
    image_width == 0U || image_height == 0U || config.confidence_threshold < 0.0 ||
    config.confidence_threshold > 1.0 || config.nms_iou_threshold < 0.0 ||
    config.nms_iou_threshold > 1.0 || config.max_detections == 0U) {
    throw std::invalid_argument("invalid YOLO postprocessing configuration");
  }

  std::vector<Candidate> candidates;
  if (config.output_format == YoloOutputFormat::RAW_XYWH) {
    candidates = decode_raw(
      output, output_shape, transform, image_width, image_height, labels.size(),
      config.confidence_threshold);
    candidates = apply_nms(std::move(candidates), config.nms_iou_threshold, config.max_detections);
  } else {
    candidates = decode_end_to_end(
      output, output_shape, transform, image_width, image_height, config.confidence_threshold);
    candidates = take_highest_scoring(std::move(candidates), config.max_detections);
  }

  std::vector<Detection> detections;
  detections.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (
      !valid_final_candidate(candidate, image_width, image_height) ||
      (!labels.empty() && candidate.class_index >= labels.size())) {
      continue;
    }
    const auto class_id = candidate.class_index < labels.size()
                            ? labels[candidate.class_index]
                            : std::to_string(candidate.class_index);
    if (class_id.empty()) {
      continue;
    }
    Detection detection;
    detection.center_x = (candidate.left + candidate.right) / 2.0;
    detection.center_y = (candidate.top + candidate.bottom) / 2.0;
    detection.size_x = candidate.right - candidate.left;
    detection.size_y = candidate.bottom - candidate.top;
    detection.score = candidate.score;
    detection.class_id = class_id;
    detections.push_back(std::move(detection));
  }
  return detections;
}

std::vector<std::string> load_labels(const std::string & path)
{
  if (path.empty()) {
    return {};
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open detector labels: " + path);
  }

  std::vector<std::string> labels;
  std::string label;
  while (std::getline(input, label)) {
    if (!label.empty()) {
      labels.push_back(label);
    }
  }
  if (labels.empty()) {
    throw std::runtime_error("detector labels file is empty: " + path);
  }
  return labels;
}

YoloOutputFormat yolo_output_format_from_string(const std::string & value)
{
  if (value == "raw_xywh") {
    return YoloOutputFormat::RAW_XYWH;
  }
  if (value == "end_to_end_xyxy") {
    return YoloOutputFormat::END_TO_END_XYXY;
  }
  throw std::invalid_argument(
    "unsupported YOLO output_format '" + value + "'; expected raw_xywh or end_to_end_xyxy");
}

}  // namespace perception_detector
