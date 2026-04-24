#include <memory>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"

namespace geometry {
namespace {

using Config = FootprintClearingPrism::Config;
using Point = FootprintClearingPrism::Point;

geometry_msgs::Quaternion YawToQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

Config MakeValidConfig() {
  Config config;
  config.enable = true;
  config.publish_visualization = false;
  config.min_z = 0.1;
  config.max_z = 1.0;
  config.footprint_points = {
      Point(0.0, 0.2),
      Point(0.4, 0.2),
      Point(0.4, -0.2),
      Point(0.0, -0.2),
  };
  return config;
}

TEST(FootprintClearingPrismTest, IsInsideRequiresHeightAndXYContainment) {
  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  prism->SetPosition(origin);
  prism->SetOrientation(YawToQuaternion(0.0));
  prism->TransformModel();

  EXPECT_TRUE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 0.5)));
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 1.1)));
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.3, 0.5)));
}

TEST(FootprintClearingPrismTest, TransformModelTracksRobotYawAndTranslation) {
  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point position;
  position.x = 5.0;
  position.y = -2.0;
  position.z = 0.0;
  prism->SetPosition(position);
  prism->SetOrientation(YawToQuaternion(M_PI / 2.0));
  prism->TransformModel();

  EXPECT_TRUE(prism->IsInside(openvdb::Vec3d(5.0, -1.8, 0.5)));
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(5.3, -2.0, 0.5)));
}

TEST(FootprintClearingPrismTest, LoadConfigFromParams) {
  ros::NodeHandle nh("/test_footprint_clearing_prism");
  nh.setParam("enable", true);
  nh.setParam("publish_visualization", false);
  nh.setParam("min_z", 0.0);
  nh.setParam("max_z", 1.2);
  nh.setParam("point1", std::vector<double>{0.0, 0.3});
  nh.setParam("point2", std::vector<double>{0.5, 0.2});
  nh.setParam("point3", std::vector<double>{0.5, -0.2});
  nh.setParam("point4", std::vector<double>{0.0, -0.3});

  std::optional<Config> config = FootprintClearingPrism::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->enable);
  EXPECT_DOUBLE_EQ(config->min_z, 0.0);
  EXPECT_DOUBLE_EQ(config->max_z, 1.2);
  ASSERT_EQ(config->footprint_points.size(), 4U);
}

TEST(FootprintClearingPrismTest, DisabledPrismAlwaysReturnsFalse) {
  Config config = MakeValidConfig();
  config.enable = false;

  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(config, nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  prism->SetPosition(origin);
  prism->SetOrientation(YawToQuaternion(0.0));
  prism->TransformModel();

  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 0.5)));
}

TEST(FootprintClearingPrismTest, CreateReturnsNullptrForInvalidConfig) {
  Config config;
  config.enable = true;
  config.min_z = 1.0;
  config.max_z = 0.5;  // max_z < min_z is invalid.
  config.footprint_points = {
      Point(0.0, 0.2),
      Point(0.4, 0.2),
      Point(0.4, -0.2),
  };

  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(config, nh);
  EXPECT_EQ(prism, nullptr);
}

TEST(FootprintClearingPrismTest, ConfigWithFewerThanThreePointsIsInvalid) {
  Config config;
  config.enable = true;
  config.min_z = 0.0;
  config.max_z = 1.0;
  config.footprint_points = {
      Point(0.0, 0.0),
      Point(1.0, 0.0),
  };

  EXPECT_FALSE(config.IsValid());
}

TEST(FootprintClearingPrismTest, PointBelowMinZIsOutside) {
  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  prism->SetPosition(origin);
  prism->SetOrientation(YawToQuaternion(0.0));
  prism->TransformModel();

  // min_z in MakeValidConfig() is 0.1
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 0.05)));
}

TEST(FootprintClearingPrismTest, PointOutsidePolygonXYIsOutside) {
  ros::NodeHandle nh;
  std::unique_ptr<FootprintClearingPrism> prism =
      FootprintClearingPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  prism->SetPosition(origin);
  prism->SetOrientation(YawToQuaternion(0.0));
  prism->TransformModel();

  // x = -0.1 is behind the polygon (which starts at x = 0.0)
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(-0.1, 0.0, 0.5)));
  // x = 0.5 is past the polygon (which ends at x = 0.4)
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.5, 0.0, 0.5)));
}

}  // namespace
}  // namespace geometry

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_footprint_clearing_prism");
  return RUN_ALL_TESTS();
}
