/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2025-2026, Bear Robotics, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 2.1 of the
 *  License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301 USA
 *
 * Author: Seung-Hun (Hoon) Han (seunghun.han@bearrobotics.ai)
 *********************************************************************/

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp"

namespace geometry {
namespace {

using PrismPoint = FootprintClearingPrism::Point;

FootprintClearingPrism::Config MakeValidPrismConfig() {
  FootprintClearingPrism::Config config;
  config.enable = true;
  config.publish_visualization = false;
  config.min_z = 0.1;
  config.max_z = 1.0;
  config.footprint_points = {
      PrismPoint(0.0, 0.2),
      PrismPoint(0.4, 0.2),
      PrismPoint(0.4, -0.2),
      PrismPoint(0.0, -0.2),
  };
  return config;
}

InterSensorDecayPrism::Config MakeValidConfig() {
  InterSensorDecayPrism::Config config;
  config.prism_config = MakeValidPrismConfig();
  config.decay_acceleration_factor = 2.0;
  return config;
}

geometry_msgs::Quaternion YawToQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

TEST(InterSensorDecayPrismTest, CreateReturnsNonNullForValidConfig) {
  ros::NodeHandle nh;
  auto prism = InterSensorDecayPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);
}

TEST(InterSensorDecayPrismTest, CreateReturnsNullForInvalidConfig) {
  ros::NodeHandle nh;
  InterSensorDecayPrism::Config config;
  config.prism_config.enable = true;
  config.prism_config.min_z = 0.0;
  config.prism_config.max_z = 1.0;
  // No footprint points — invalid
  config.decay_acceleration_factor = 2.0;
  auto prism = InterSensorDecayPrism::Create(config, nh);
  EXPECT_EQ(prism, nullptr);
}

TEST(InterSensorDecayPrismTest,
     CreateReturnsNullForNonPositiveAccelerationFactor) {
  ros::NodeHandle nh;
  auto config = MakeValidConfig();
  config.decay_acceleration_factor = 0.0;
  auto prism = InterSensorDecayPrism::Create(config, nh);
  EXPECT_EQ(prism, nullptr);
}

TEST(InterSensorDecayPrismTest, IsInsideDelegatesToInnerPrism) {
  ros::NodeHandle nh;
  auto prism = InterSensorDecayPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  prism->SetPosition(origin);
  prism->SetOrientation(YawToQuaternion(0.0));
  prism->TransformModel();

  // Inside polygon and height range
  EXPECT_TRUE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 0.5)));
  // Outside height range
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.0, 1.1)));
  // Outside polygon
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(0.2, 0.3, 0.5)));
}

TEST(InterSensorDecayPrismTest, TransformModelTracksRobotPose) {
  ros::NodeHandle nh;
  auto prism = InterSensorDecayPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);

  geometry_msgs::Point position;
  position.x = 5.0;
  position.y = -2.0;
  position.z = 0.0;
  prism->SetPosition(position);
  prism->SetOrientation(YawToQuaternion(M_PI / 2.0));
  prism->TransformModel();

  // After 90 degree rotation + translation, x-forward becomes y-forward
  EXPECT_TRUE(prism->IsInside(openvdb::Vec3d(5.0, -1.8, 0.5)));
  EXPECT_FALSE(prism->IsInside(openvdb::Vec3d(5.3, -2.0, 0.5)));
}

TEST(InterSensorDecayPrismTest, DecayAccelerationFactorReturnsConfigValue) {
  ros::NodeHandle nh;
  auto prism = InterSensorDecayPrism::Create(MakeValidConfig(), nh);
  ASSERT_NE(prism, nullptr);
  EXPECT_DOUBLE_EQ(prism->decay_acceleration_factor(), 2.0);
}

TEST(InterSensorDecayPrismTest, LoadConfigFromParams) {
  ros::NodeHandle nh("/test_inter_sensor_decay_prism");
  nh.setParam("enable", true);
  nh.setParam("publish_visualization", false);
  nh.setParam("min_z", 0.0);
  nh.setParam("max_z", 1.2);
  nh.setParam("decay_acceleration_factor", 5.0);
  nh.setParam("point1", std::vector<double>{0.0, 0.3});
  nh.setParam("point2", std::vector<double>{0.5, 0.2});
  nh.setParam("point3", std::vector<double>{0.5, -0.2});
  nh.setParam("point4", std::vector<double>{0.0, -0.3});

  auto config = InterSensorDecayPrism::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->prism_config.enable);
  EXPECT_DOUBLE_EQ(config->prism_config.min_z, 0.0);
  EXPECT_DOUBLE_EQ(config->prism_config.max_z, 1.2);
  EXPECT_DOUBLE_EQ(config->decay_acceleration_factor, 5.0);
  ASSERT_EQ(config->prism_config.footprint_points.size(), 4u);
}

}  // namespace
}  // namespace geometry

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_inter_sensor_decay_prism");
  return RUN_ALL_TESTS();
}
