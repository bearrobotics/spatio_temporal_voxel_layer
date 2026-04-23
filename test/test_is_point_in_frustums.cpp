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
 * Author: Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *********************************************************************/

#include <openvdb/openvdb.h>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

using spatio_temporal_voxel_layer::test_utils::MakeTestGrid;

observation::MeasurementReading MakeReading(
    ros::NodeHandle& frustum_nh, const std::string& sensor_name,
    const ros::Time& time, const geometry_msgs::Point& origin,
    const geometry_msgs::Quaternion& orientation) {
  observation::MeasurementReading reading;
  reading._sensor_name = sensor_name;
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>();
  reading._cloud->header.stamp = time;
  reading._origin = origin;
  reading._orientation = orientation;
  reading._frustrum_factory =
      FrustumFactoryFactory::CreateFrustumFactory(frustum_nh);
  reading._marking = false;
  reading._clearing = false;
  return reading;
}

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion MakeIdentityQuaternion() {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

class IsPointInFrustumsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    grid_ = MakeTestGrid();
    frustum_nh_ = ros::NodeHandle("/is_point_test/frustum");
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
  ros::NodeHandle frustum_nh_;
};

TEST_F(IsPointInFrustumsTest, EmptyReadings_ReturnsFalse) {
  // No sensors have been registered
  openvdb::Vec3d any_point(1.0, 1.0, 1.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(any_point));
}

TEST_F(IsPointInFrustumsTest, PointInsideSingleFrustum) {
  // Sensor at origin, pointing along +Z axis
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point directly in front, within FOV and range (min_z=0.1, max_z=5.0)
  openvdb::Vec3d point_inside(0.0, 0.0, 2.0);
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_inside));
}

TEST_F(IsPointInFrustumsTest, PointOutsideSingleFrustum_Behind) {
  // Sensor at origin, pointing along +Z axis
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point behind the sensor (negative Z)
  openvdb::Vec3d point_behind(0.0, 0.0, -2.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_behind));
}

TEST_F(IsPointInFrustumsTest, PointOutsideSingleFrustum_TooFar) {
  // Sensor at origin, pointing along +Z axis
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point beyond max range (max_z=5.0)
  openvdb::Vec3d point_far(0.0, 0.0, 10.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_far));
}

TEST_F(IsPointInFrustumsTest, PointOutsideSingleFrustum_TooClose) {
  // Sensor at origin, pointing along +Z axis
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point closer than min range (min_z=0.1)
  openvdb::Vec3d point_close(0.0, 0.0, 0.05);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_close));
}

TEST_F(IsPointInFrustumsTest, PointOutsideSingleFrustum_OutsideHFOV) {
  // Sensor at origin, pointing along +Z axis
  // horizontal_fov_angle: 1.2 radians (~69 degrees total, ~34.5 degrees each
  // side)
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point way off to the side (outside horizontal FOV)
  // At z=2.0, with hFOV of 1.2 rad, max x is roughly 2.0 * tan(0.6) = 1.37
  openvdb::Vec3d point_side(5.0, 0.0, 2.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_side));
}

TEST_F(IsPointInFrustumsTest, PointInsideOneOfMultipleFrustums) {
  // Two sensors at different positions
  auto reading_a =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  auto reading_b =
      MakeReading(frustum_nh_, "sensor_b", ros::Time(1.0),
                  MakePoint(10.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_a);
  grid_->UpdateLastReadings(reading_b);

  // Point only in sensor_b's FOV
  openvdb::Vec3d point_near_b(10.0, 0.0, 2.0);
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_near_b));

  // Point only in sensor_a's FOV
  openvdb::Vec3d point_near_a(0.0, 0.0, 2.0);
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_near_a));
}

TEST_F(IsPointInFrustumsTest, PointOutsideAllFrustums) {
  // Two sensors at different positions
  auto reading_a =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  auto reading_b =
      MakeReading(frustum_nh_, "sensor_b", ros::Time(1.0),
                  MakePoint(10.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_a);
  grid_->UpdateLastReadings(reading_b);

  // Point not in either sensor's FOV
  openvdb::Vec3d point_between(5.0, 5.0, 2.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_between));
}

TEST_F(IsPointInFrustumsTest, PointAtMinRange) {
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point exactly at min_z (0.1)
  openvdb::Vec3d point_at_min(0.0, 0.0, 0.1);
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_at_min));
}

TEST_F(IsPointInFrustumsTest, PointBeyondMaxRange) {
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  grid_->UpdateLastReadings(reading);

  // Point well beyond max range (max_z=5.0)
  openvdb::Vec3d point_beyond_max(0.0, 0.0, 10.0);
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_beyond_max));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_is_point_in_frustums");
  return RUN_ALL_TESTS();
}
