#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"

// ============================================================================
// FrustumFactoryFactory Tests
// ============================================================================

TEST(FrustumFactoryTest, CreateDepthCameraFrustumWithValidParams) {
  ros::NodeHandle nh("/frustum_factory_test/depth_camera_valid");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  ASSERT_NE(factory, nullptr);
  auto frustum = factory();
  EXPECT_NE(frustum, nullptr);
}

TEST(FrustumFactoryTest, CreateFrustumWithMissingType) {
  ros::NodeHandle nh("/frustum_factory_test/missing_type");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateFrustumWithUnknownType) {
  ros::NodeHandle nh("/frustum_factory_test/unknown_type");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateFrustumWithEmptyType) {
  ros::NodeHandle nh("/frustum_factory_test/empty_type");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateDepthCameraFrustumMissingVerticalFov) {
  ros::NodeHandle nh("/frustum_factory_test/missing_vertical_fov");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateDepthCameraFrustumMissingHorizontalFov) {
  ros::NodeHandle nh("/frustum_factory_test/missing_horizontal_fov");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateDepthCameraFrustumMissingMinZ) {
  ros::NodeHandle nh("/frustum_factory_test/missing_min_z");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

TEST(FrustumFactoryTest, CreateDepthCameraFrustumMissingMaxZ) {
  ros::NodeHandle nh("/frustum_factory_test/missing_max_z");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  EXPECT_EQ(factory, nullptr);
}

// ============================================================================
// DepthCameraFrustum::LoadConfig Tests
// ============================================================================

TEST(DepthCameraFrustumTest, LoadConfigWithValidParams) {
  ros::NodeHandle nh("/frustum_factory_test/depth_camera_valid");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->vertical_fov_angle, 1.0);
  EXPECT_DOUBLE_EQ(config->horizontal_fov_angle, 1.2);
  EXPECT_DOUBLE_EQ(config->min_distance, 0.1);
  EXPECT_DOUBLE_EQ(config->max_distance, 5.0);
}

TEST(DepthCameraFrustumTest, LoadConfigMissingVerticalFov) {
  ros::NodeHandle nh("/frustum_factory_test/missing_vertical_fov");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(DepthCameraFrustumTest, LoadConfigMissingHorizontalFov) {
  ros::NodeHandle nh("/frustum_factory_test/missing_horizontal_fov");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(DepthCameraFrustumTest, LoadConfigMissingMinZ) {
  ros::NodeHandle nh("/frustum_factory_test/missing_min_z");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(DepthCameraFrustumTest, LoadConfigMissingMaxZ) {
  ros::NodeHandle nh("/frustum_factory_test/missing_max_z");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(DepthCameraFrustumTest, LoadConfigWithZeroValues) {
  ros::NodeHandle nh("/frustum_factory_test/zero_values");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->vertical_fov_angle, 0.0);
  EXPECT_DOUBLE_EQ(config->horizontal_fov_angle, 0.0);
  EXPECT_DOUBLE_EQ(config->min_distance, 0.0);
  EXPECT_DOUBLE_EQ(config->max_distance, 0.0);
}

TEST(DepthCameraFrustumTest, LoadConfigWithNegativeValues) {
  ros::NodeHandle nh("/frustum_factory_test/negative_values");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->vertical_fov_angle, -1.0);
  EXPECT_DOUBLE_EQ(config->horizontal_fov_angle, -1.2);
  EXPECT_DOUBLE_EQ(config->min_distance, -0.1);
  EXPECT_DOUBLE_EQ(config->max_distance, -5.0);
}

TEST(DepthCameraFrustumTest, LoadConfigWithMinGreaterThanMax) {
  ros::NodeHandle nh("/frustum_factory_test/min_greater_than_max");
  auto config = geometry::DepthCameraFrustum::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_GT(config->min_distance, config->max_distance);
}

TEST(FrustumFactoryTest, FactoryCanCreateMultipleFrustums) {
  ros::NodeHandle nh("/frustum_factory_test/depth_camera_valid");
  auto factory = FrustumFactoryFactory::CreateFrustumFactory(nh);
  ASSERT_NE(factory, nullptr);

  auto frustum1 = factory();
  auto frustum2 = factory();
  EXPECT_NE(frustum1, nullptr);
  EXPECT_NE(frustum2, nullptr);
  EXPECT_NE(frustum1, frustum2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_frustum_factory");
  return RUN_ALL_TESTS();
}
