#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "perception_detection_depth_projection/depth_projector.hpp"

namespace perception_detection_depth_projection
{
namespace
{

std::vector<float> flat_depth_image(float depth_m)
{
  return std::vector<float>(100U, depth_m);
}

DepthImageView view_of(const std::vector<float> & values)
{
  return DepthImageView::create(
           10U, 10U, 10U * sizeof(float), DepthEncoding::FLOAT32_METERS, false,
           reinterpret_cast<const std::uint8_t *>(values.data()), values.size() * sizeof(float))
    .value();
}

const CameraIntrinsics kCamera{100.0, 100.0, 5.0, 5.0};

TEST(DepthProjectorUnitTest, ProjectsCenterBoundingBoxIntoCameraFrame)
{
  DepthProjector projector;
  const auto depth = flat_depth_image(2.0F);
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 4.0, 2.0}, view_of(depth), kCamera);

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->x_m, 0.0);
  EXPECT_DOUBLE_EQ(result->y_m, 0.0);
  EXPECT_DOUBLE_EQ(result->z_m, 2.0);
  EXPECT_DOUBLE_EQ(result->width_m, 0.08);
  EXPECT_DOUBLE_EQ(result->height_m, 0.04);
}

TEST(DepthProjectorUnitTest, UsesMedianToRejectSingleDepthOutlier)
{
  auto depth = flat_depth_image(2.0F);
  depth[55U] = 7.0F;
  DepthProjector projector;
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 8.0, 8.0}, view_of(depth), kCamera);

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->z_m, 2.0);
}

TEST(DepthProjectorUnitTest, RejectsInsufficientValidDepth)
{
  auto depth = flat_depth_image(std::numeric_limits<float>::quiet_NaN());
  depth[55U] = 2.0F;
  DepthProjector projector;
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 4.0, 4.0}, view_of(depth), kCamera);

  EXPECT_FALSE(result);
}

TEST(DepthProjectorUnitTest, RejectsUncalibratedCamera)
{
  DepthProjector projector;
  const auto depth = flat_depth_image(2.0F);
  const auto result =
    projector.project(BoundingBox2D{5.0, 5.0, 4.0, 4.0}, view_of(depth), CameraIntrinsics{});

  EXPECT_FALSE(result);
}

TEST(DepthProjectorUnitTest, ReadsUint16MillimetresWithoutCopyingTheFrame)
{
  constexpr std::size_t width = 10U;
  constexpr std::size_t height = 10U;
  constexpr std::size_t row_step = 24U;
  std::vector<std::uint8_t> data(row_step * height, 0U);
  for (std::size_t row = 0U; row < height; ++row) {
    for (std::size_t column = 0U; column < width; ++column) {
      const auto offset = row * row_step + column * sizeof(std::uint16_t);
      data[offset] = 0xD0U;
      data[offset + 1U] = 0x07U;
    }
  }
  const auto depth = DepthImageView::create(
    width, height, row_step, DepthEncoding::UINT16_MILLIMETERS, false, data.data(), data.size());
  DepthProjector projector;

  ASSERT_TRUE(depth);
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 4.0, 4.0}, *depth, kCamera);

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->z_m, 2.0);
}

TEST(DepthImageViewUnitTest, RejectsTruncatedBufferDuringCreation)
{
  const std::vector<std::uint8_t> data(10U, 0U);

  const auto depth = DepthImageView::create(
    10U, 10U, 10U * sizeof(float), DepthEncoding::FLOAT32_METERS, false, data.data(), data.size());

  EXPECT_FALSE(depth);
}

TEST(DepthImageViewUnitTest, ReadsBigEndianUint16Millimetres)
{
  const std::vector<std::uint8_t> data{0x07U, 0xD0U};
  const auto depth = DepthImageView::create(
    1U, 1U, data.size(), DepthEncoding::UINT16_MILLIMETERS, true, data.data(), data.size());

  ASSERT_TRUE(depth);
  const auto value = depth->value_m(0U, 0U);
  ASSERT_TRUE(value);
  EXPECT_FLOAT_EQ(*value, 2.0F);
}

TEST(DepthImageViewUnitTest, ReadsBigEndianFloatMetres)
{
  const std::vector<std::uint8_t> data{0x40U, 0x00U, 0x00U, 0x00U};
  const auto depth = DepthImageView::create(
    1U, 1U, data.size(), DepthEncoding::FLOAT32_METERS, true, data.data(), data.size());

  ASSERT_TRUE(depth);
  const auto value = depth->value_m(0U, 0U);
  ASSERT_TRUE(value);
  EXPECT_FLOAT_EQ(*value, 2.0F);
}

}  // namespace
}  // namespace perception_detection_depth_projection
