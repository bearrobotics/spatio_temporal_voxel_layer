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
#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"

namespace {

TEST(CircleIsInsideTest, PointAtCenter) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(1.0, 2.0);
  circle.radius = 0.5f;

  openvdb::Vec3d point(1.0, 2.0, 0.0);
  EXPECT_TRUE(circle.IsInside(point));
}

TEST(CircleIsInsideTest, PointOnBoundary) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(0.0, 0.0);
  circle.radius = 1.0f;

  openvdb::Vec3d point(1.0, 0.0, 0.0);
  EXPECT_TRUE(circle.IsInside(point));
}

TEST(CircleIsInsideTest, PointInside) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(0.0, 0.0);
  circle.radius = 1.0f;

  openvdb::Vec3d point(0.5, 0.5, 0.0);
  EXPECT_TRUE(circle.IsInside(point));
}

TEST(CircleIsInsideTest, PointOutside) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(0.0, 0.0);
  circle.radius = 1.0f;

  openvdb::Vec3d point(2.0, 0.0, 0.0);
  EXPECT_FALSE(circle.IsInside(point));
}

TEST(CircleIsInsideTest, ZeroRadius) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(1.0, 1.0);
  circle.radius = 0.0f;

  openvdb::Vec3d at_center(1.0, 1.0, 0.0);
  openvdb::Vec3d nearby(1.001, 1.0, 0.0);
  EXPECT_TRUE(circle.IsInside(at_center));
  EXPECT_FALSE(circle.IsInside(nearby));
}

TEST(CircleIsInsideTest, ZIgnored) {
  DynamicObstacleTracker::Circle circle;
  circle.center = Eigen::Vector2d(0.0, 0.0);
  circle.radius = 1.0f;

  openvdb::Vec3d point_low_z(0.5, 0.5, -100.0);
  openvdb::Vec3d point_high_z(0.5, 0.5, 100.0);
  EXPECT_TRUE(circle.IsInside(point_low_z));
  EXPECT_TRUE(circle.IsInside(point_high_z));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
