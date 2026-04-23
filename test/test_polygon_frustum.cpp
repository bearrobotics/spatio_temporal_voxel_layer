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
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"

namespace {

DynamicObstacleReading::Polygon MakeSquarePolygon(float cx, float cy,
                                                  float size) {
  DynamicObstacleReading::Polygon polygon;
  float half = size / 2.0f;
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy - half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx + half, cy - half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx + half, cy + half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy + half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy - half));
  return polygon;
}

TEST(PolygonIsInsideTest, PointInside) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point(0.0, 0.0, 0.0);
  EXPECT_TRUE(polygon.IsInside(point));
}

TEST(PolygonIsInsideTest, PointOutside) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point(5.0, 5.0, 0.0);
  EXPECT_FALSE(polygon.IsInside(point));
}

TEST(PolygonIsInsideTest, ZIgnored) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point_high_z(0.0, 0.0, 1000.0);
  EXPECT_TRUE(polygon.IsInside(point_high_z));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
