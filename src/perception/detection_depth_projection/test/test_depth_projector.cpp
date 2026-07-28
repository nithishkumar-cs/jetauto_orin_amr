#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "perception_detection_depth_projection/depth_projector.hpp"

namespace perception_detection_depth_projection
{
namespace
{

DepthImage flat_depth_image(float depth_m)
{
  return DepthImage{10U, 10U, std::vector<float>(100U, depth_m)};
}

const CameraIntrinsics kCamera{100.0, 100.0, 5.0, 5.0};

TEST(DepthProjectorTest, ProjectsCenterBoundingBoxIntoCameraFrame)
{
  DepthProjector projector;
  const auto result =
    projector.project(BoundingBox2D{5.0, 5.0, 4.0, 2.0}, flat_depth_image(2.0F), kCamera);

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->x_m, 0.0);
  EXPECT_DOUBLE_EQ(result->y_m, 0.0);
  EXPECT_DOUBLE_EQ(result->z_m, 2.0);
  EXPECT_DOUBLE_EQ(result->width_m, 0.08);
  EXPECT_DOUBLE_EQ(result->height_m, 0.04);
}

TEST(DepthProjectorTest, UsesMedianToRejectSingleDepthOutlier)
{
  auto depth = flat_depth_image(2.0F);
  depth.values_m[55U] = 7.0F;
  DepthProjector projector;
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 8.0, 8.0}, depth, kCamera);

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->z_m, 2.0);
}

TEST(DepthProjectorTest, RejectsInsufficientValidDepth)
{
  auto depth = flat_depth_image(std::numeric_limits<float>::quiet_NaN());
  depth.values_m[55U] = 2.0F;
  DepthProjector projector;
  const auto result = projector.project(BoundingBox2D{5.0, 5.0, 4.0, 4.0}, depth, kCamera);

  EXPECT_FALSE(result);
}

TEST(DepthProjectorTest, RejectsUncalibratedCamera)
{
  DepthProjector projector;
  const auto result = projector.project(
    BoundingBox2D{5.0, 5.0, 4.0, 4.0}, flat_depth_image(2.0F), CameraIntrinsics{});

  EXPECT_FALSE(result);
}

}  // namespace
}  // namespace perception_detection_depth_projection
