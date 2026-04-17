#include <geometry_msgs/Quaternion.h>

#include <cmath>
#include <spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace geometry {
namespace {

using Point = FootprintFrustum::Point;
using Config = FootprintFrustum::Config;

// Helper to create a diamond-shaped footprint centered at origin
std::vector<Point> MakeDiamondFootprint(double size = 1.0) {
  return {Point(size, 0.0), Point(0.0, size), Point(-size, 0.0),
          Point(0.0, -size)};
}

// Helper to create a triangle footprint
std::vector<Point> MakeTriangleFootprint() {
  return {Point(1.0, 0.0), Point(0.0, 1.0), Point(-1.0, 0.0)};
}

// Helper to convert yaw angle to quaternion
geometry_msgs::Quaternion YawToQuaternion(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

//------------------------------------------------------------------------------
// Config/Param Loading Tests
//------------------------------------------------------------------------------

TEST(FootprintFrustumTest, LoadValidConfig) {
  ros::NodeHandle nh("/footprint_frustum/valid");
  auto config = FootprintFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->enable);
  EXPECT_EQ(config->footprint_points.size(), 4);
}

TEST(FootprintFrustumTest, LoadDisabledConfig) {
  ros::NodeHandle nh("/footprint_frustum/disabled");
  auto config = FootprintFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_FALSE(config->enable);
  EXPECT_EQ(config->footprint_points.size(), 3);
}

TEST(FootprintFrustumTest, LoadMissingEnableParam) {
  ros::NodeHandle nh("/footprint_frustum/nonexistent");
  auto config = FootprintFrustum::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(FootprintFrustumTest, LoadEmptyFootprint) {
  ros::NodeHandle nh("/footprint_frustum/empty_footprint");
  auto config = FootprintFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->enable);
  EXPECT_EQ(config->footprint_points.size(), 0);
}

//------------------------------------------------------------------------------
// Config Validation Tests (IsValid)
//------------------------------------------------------------------------------

TEST(FootprintFrustumTest, IsValidWithValidPolygon) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeTriangleFootprint();
  EXPECT_TRUE(config.IsValid());
}

TEST(FootprintFrustumTest, IsValidWithFourPoints) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  EXPECT_TRUE(config.IsValid());
}

TEST(FootprintFrustumTest, IsValidWithTooFewPoints) {
  Config config;
  config.enable = true;
  config.footprint_points = {Point(1.0, 0.0), Point(0.0, 1.0)};
  EXPECT_FALSE(config.IsValid());
}

TEST(FootprintFrustumTest, IsValidWithNoPoints) {
  Config config;
  config.enable = true;
  config.footprint_points = {};
  EXPECT_FALSE(config.IsValid());
}

//------------------------------------------------------------------------------
// Factory Method Tests (Create)
//------------------------------------------------------------------------------

TEST(FootprintFrustumTest, CreateWithValidConfig) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  EXPECT_NE(frustum, nullptr);
}

TEST(FootprintFrustumTest, CreateWithInvalidConfig) {
  Config config;
  config.enable = true;
  config.footprint_points = {Point(1.0, 0.0), Point(0.0, 1.0)};
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  EXPECT_EQ(frustum, nullptr);
}

//------------------------------------------------------------------------------
// IsInside Tests
//------------------------------------------------------------------------------

TEST(FootprintFrustumTest, IsInsideWhenDisabled) {
  Config config;
  config.enable = false;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  frustum->SetPosition(origin);
  frustum->TransformModel();

  // Point clearly inside polygon, but should return false because disabled
  openvdb::Vec3d inside_pt(0.0, 0.0, 0.0);
  EXPECT_FALSE(frustum->IsInside(inside_pt));
}

TEST(FootprintFrustumTest, IsInsidePointInPolygon) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  frustum->SetPosition(origin);
  frustum->TransformModel();

  // Point at center should be inside
  openvdb::Vec3d center_pt(0.0, 0.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(center_pt));

  // Point slightly off-center should still be inside
  openvdb::Vec3d offset_pt(0.1, 0.1, 0.0);
  EXPECT_TRUE(frustum->IsInside(offset_pt));
}

TEST(FootprintFrustumTest, IsInsidePointOutsidePolygon) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  frustum->SetPosition(origin);
  frustum->TransformModel();

  // Point clearly outside
  openvdb::Vec3d outside_pt(2.0, 2.0, 0.0);
  EXPECT_FALSE(frustum->IsInside(outside_pt));

  // Point just outside the diamond (corner region)
  openvdb::Vec3d corner_outside(0.8, 0.8, 0.0);
  EXPECT_FALSE(frustum->IsInside(corner_outside));
}

TEST(FootprintFrustumTest, IsInsideIgnoresZComponent) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  frustum->SetPosition(origin);
  frustum->TransformModel();

  // Z component should be ignored - point at center with various Z values
  openvdb::Vec3d pt_z0(0.0, 0.0, 0.0);
  openvdb::Vec3d pt_z1(0.0, 0.0, 1.0);
  openvdb::Vec3d pt_z_neg(0.0, 0.0, -5.0);

  EXPECT_TRUE(frustum->IsInside(pt_z0));
  EXPECT_TRUE(frustum->IsInside(pt_z1));
  EXPECT_TRUE(frustum->IsInside(pt_z_neg));
}

//------------------------------------------------------------------------------
// TransformModel Tests
//------------------------------------------------------------------------------

TEST(FootprintFrustumTest, TransformModelWithTranslation) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  // Translate to (5, 5)
  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point pos;
  pos.x = 5.0;
  pos.y = 5.0;
  pos.z = 0.0;
  frustum->SetPosition(pos);
  frustum->TransformModel();

  // Original center (0,0) should now be at (5,5)
  openvdb::Vec3d new_center(5.0, 5.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(new_center));

  // Original center location should now be outside
  openvdb::Vec3d original_center(0.0, 0.0, 0.0);
  EXPECT_FALSE(frustum->IsInside(original_center));
}

TEST(FootprintFrustumTest, TransformModelWithYaw) {
  Config config;
  config.enable = true;
  // Use a simple square for easier rotation verification
  config.footprint_points = {Point(1.0, 0.0), Point(0.0, 1.0), Point(-1.0, 0.0),
                             Point(0.0, -1.0)};
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  // Rotate 90 degrees (pi/2)
  frustum->SetOrientation(YawToQuaternion(M_PI / 2.0));
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  frustum->SetPosition(origin);
  frustum->TransformModel();

  // Center should still be inside
  openvdb::Vec3d center(0.0, 0.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(center));

  // Point at (0.5, 0) should still be inside after any rotation
  // because it's closer to center than the vertices
  openvdb::Vec3d inner_pt(0.3, 0.3, 0.0);
  EXPECT_TRUE(frustum->IsInside(inner_pt));
}

TEST(FootprintFrustumTest, TransformModelCombinedTranslationAndRotation) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  // Translate to (10, 0) and rotate 180 degrees
  frustum->SetOrientation(YawToQuaternion(M_PI));
  geometry_msgs::Point pos;
  pos.x = 10.0;
  pos.y = 0.0;
  pos.z = 0.0;
  frustum->SetPosition(pos);
  frustum->TransformModel();

  // New center should be at translated position
  openvdb::Vec3d new_center(10.0, 0.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(new_center));

  // Original position should be outside
  openvdb::Vec3d original_center(0.0, 0.0, 0.0);
  EXPECT_FALSE(frustum->IsInside(original_center));
}

TEST(FootprintFrustumTest, MultipleTransformCalls) {
  Config config;
  config.enable = true;
  config.footprint_points = MakeDiamondFootprint();
  ros::NodeHandle nh;
  auto frustum = FootprintFrustum::Create(config, nh);
  ASSERT_NE(frustum, nullptr);

  // First transform
  frustum->SetOrientation(YawToQuaternion(0.0));
  geometry_msgs::Point pos1;
  pos1.x = 5.0;
  pos1.y = 0.0;
  pos1.z = 0.0;
  frustum->SetPosition(pos1);
  frustum->TransformModel();

  openvdb::Vec3d pt1(5.0, 0.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(pt1));

  // Second transform - should use original polygon, not accumulated
  geometry_msgs::Point pos2;
  pos2.x = -5.0;
  pos2.y = 0.0;
  pos2.z = 0.0;
  frustum->SetPosition(pos2);
  frustum->TransformModel();

  openvdb::Vec3d pt2(-5.0, 0.0, 0.0);
  EXPECT_TRUE(frustum->IsInside(pt2));

  // Previous position should now be outside
  EXPECT_FALSE(frustum->IsInside(pt1));
}

}  // namespace
}  // namespace geometry

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_footprint_frustum");
  return RUN_ALL_TESTS();
}
