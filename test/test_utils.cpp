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

#include "test/test_utils.h"

#include <openvdb/openvdb.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace spatio_temporal_voxel_layer {
namespace test_utils {

std::unique_ptr<geometry::FootprintFrustum> MakeMockFootprintFrustum() {
  ros::NodeHandle nh;
  geometry::FootprintFrustum::Config config;
  config.enable = false;
  config.footprint_points = {geometry::FootprintFrustum::Point(0.0, 0.0),
                             geometry::FootprintFrustum::Point(0.1, 0.0),
                             geometry::FootprintFrustum::Point(0.0, 0.1)};
  return geometry::FootprintFrustum::Create(config, nh);
}

std::unique_ptr<DynamicObstacleTracker> MakeMockDynamicObstacleTracker() {
  ros::NodeHandle nh;
  DynamicObstacleTracker::Config config;
  config.enable = false;
  config.activation_velocity_threshold = 0.1;
  config.min_distance_between_readings_threshold = 0.01;
  config.interpolation_in_future = 0.1;
  config.interpolation_in_past = 0.1;
  config.stale_time_threshold = 1.0;
  config.inflation_radius_factor = 1.0;
  config.number_of_interpolation_circles = 1;
  config.past_time_window = 1.0;
  config.random_walk_probability_limit = 0.5;
  config.max_obstacle_radius = 1.0;
  config.seconds_since_last_random_walk = 1.0;
  config.publish_visualization = false;
  return DynamicObstacleTracker::Create(config, nh);
}

std::unique_ptr<RobotMotionTracker> MakeMockRobotMotionTracker() {
  ros::NodeHandle nh;
  RobotMotionTracker::Config config;
  config.enable = false;
  config.activation_velocity_threshold = 0.1;
  config.min_distance_between_readings_threshold = 0.01;
  config.stale_time_threshold = 1.0;
  config.inflation_radius_factor = 1.0;
  config.number_of_interpolation_circles = 1;
  config.past_time_window = 1.0;
  config.publish_visualization = false;
  return RobotMotionTracker::Create(config, nh);
}

std::unique_ptr<geometry::InterSensorDecayPrism>
MakeMockInterSensorDecayPrism() {
  return nullptr;
}

std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> MakeTestGrid(
    std::unique_ptr<geometry::InterSensorDecayPrism> inter_sensor_decay_prism,
    std::unique_ptr<geometry::FootprintClearingPrism>
        front_blind_spot_clearing_prism) {
  openvdb::initialize();
  return std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(
      kVoxelSize, kBackgroundValue, kDecayModel, kVoxelDecay, kPubVoxels,
      MakeMockFootprintFrustum(), std::move(inter_sensor_decay_prism),
      std::move(front_blind_spot_clearing_prism),
      MakeMockDynamicObstacleTracker(), MakeMockRobotMotionTracker());
}

sensor_msgs::PointCloud2 MakePointCloud(
    const std::vector<geometry_msgs::Point>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "map";

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& pt : points) {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

}  // namespace test_utils
}  // namespace spatio_temporal_voxel_layer
