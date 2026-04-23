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

class UpdateLastReadingsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    grid_ = MakeTestGrid();
    frustum_nh_ = ros::NodeHandle("/update_readings_test/frustum");
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
  ros::NodeHandle frustum_nh_;
};

TEST_F(UpdateLastReadingsTest, AddFirstReading) {
  auto reading =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading);

  // Verify the reading was added by checking IsPointInLastSensorFrustums
  // A point directly in front of the sensor (within FOV) should be inside
  openvdb::Vec3d point_in_front(0.0, 0.0, 1.0);
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_in_front));
}

TEST_F(UpdateLastReadingsTest, AddMultipleSensors) {
  auto reading_a =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  auto reading_b =
      MakeReading(frustum_nh_, "sensor_b", ros::Time(1.0),
                  MakePoint(5.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_a);
  grid_->UpdateLastReadings(reading_b);

  // Both sensor FOVs should be active
  openvdb::Vec3d point_near_a(0.0, 0.0, 1.0);
  openvdb::Vec3d point_near_b(5.0, 0.0, 1.0);

  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_near_a));
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_near_b));
}

TEST_F(UpdateLastReadingsTest, UpdateExistingSensor_NewerTime) {
  auto reading_old =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  // New reading at different position with newer time
  auto reading_new =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(2.0),
                  MakePoint(10.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_old);
  grid_->UpdateLastReadings(reading_new);

  // The new position should be active, old position should not be
  openvdb::Vec3d point_at_old_pos(0.0, 0.0, 1.0);
  openvdb::Vec3d point_at_new_pos(10.0, 0.0, 1.0);

  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_at_old_pos));
  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_at_new_pos));
}

TEST_F(UpdateLastReadingsTest, UpdateExistingSensor_OlderTime) {
  auto reading_new =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(2.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  // Older reading should be ignored
  auto reading_old =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(10.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_new);
  grid_->UpdateLastReadings(reading_old);

  // The newer position should still be active, older one ignored
  openvdb::Vec3d point_at_new_pos(0.0, 0.0, 1.0);
  openvdb::Vec3d point_at_old_pos(10.0, 0.0, 1.0);

  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_at_new_pos));
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_at_old_pos));
}

TEST_F(UpdateLastReadingsTest, UpdateExistingSensor_SameTime) {
  auto reading_first =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());
  // Same timestamp, different position - should be ignored
  auto reading_same_time =
      MakeReading(frustum_nh_, "sensor_a", ros::Time(1.0),
                  MakePoint(10.0, 0.0, 0.0), MakeIdentityQuaternion());

  grid_->UpdateLastReadings(reading_first);
  grid_->UpdateLastReadings(reading_same_time);

  // First position should still be active
  openvdb::Vec3d point_at_first_pos(0.0, 0.0, 1.0);
  openvdb::Vec3d point_at_second_pos(10.0, 0.0, 1.0);

  EXPECT_TRUE(grid_->IsPointInLastSensorFrustums(point_at_first_pos));
  EXPECT_FALSE(grid_->IsPointInLastSensorFrustums(point_at_second_pos));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_update_last_readings");
  return RUN_ALL_TESTS();
}
