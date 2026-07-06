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

#include <geometry_msgs/Point.h>
#include <openvdb/openvdb.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using volume_grid::SpatioTemporalVoxelGrid;

// Query radius for locating a marked voxel by its XY position. Larger than the
// worst-case voxel-center offset within a cell (~0.036 m at a 0.05 m voxel
// size) and far smaller than the 0.5 m spacing between distinct marks.
constexpr double kQueryTolerance = 0.07;

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

observation::MeasurementReading MakeMarkingReading(
    const std::vector<geometry_msgs::Point>& points, int voxel_class) {
  observation::MeasurementReading reading;
  reading._sensor_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(points));
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  reading._voxel_class = voxel_class;
  return reading;
}

// Runs one decay/costmap-generation pass with no clearing inputs, exercising
// the same temporal-clearing path the layer drives every cycle.
void RunDecayPass(SpatioTemporalVoxelGrid* grid) {
  std::unordered_set<volume_grid::occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  grid->ClearFrustums({}, cleared_cells, dynamic_obstacle_readings,
                      robot_motion_readings);
}

// Fixture uses the default test grid (PERSISTENT decay). Clearing in these
// cases is driven explicitly, so the decay model does not matter here.
class CliffPersistenceTest : public ::testing::Test {
 protected:
  void SetUp() override { grid_ = test_utils::MakeTestGrid(); }

  void Mark(const std::vector<geometry_msgs::Point>& points, int voxel_class) {
    grid_->Mark({MakeMarkingReading(points, voxel_class)});
  }

  bool HasVoxelNear(double x, double y) {
    return !grid_->GetVoxelsAtXY(x, y, kQueryTolerance).empty();
  }

  std::unique_ptr<SpatioTemporalVoxelGrid> grid_;
};

// Footprint clearing (Pan/Backup recovery) clears a circular area regardless of
// voxel height. Permanent cliffs inside that area must be preserved while
// ordinary obstacles are cleared.
TEST_F(CliffPersistenceTest, FootprintClearPreservesCliffButClearsGeneric) {
  Mark({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kCliff);
  Mark({MakePoint(0.5, 0.0, 0.1)}, volume_grid::kGeneric);
  ASSERT_TRUE(HasVoxelNear(0.0, 0.0));
  ASSERT_TRUE(HasVoxelNear(0.5, 0.0));

  grid_->ClearCircularArea(0.25, 0.0, 1.0);

  EXPECT_TRUE(HasVoxelNear(0.0, 0.0))
      << "cliff voxel must survive footprint clearing";
  EXPECT_FALSE(HasVoxelNear(0.5, 0.0))
      << "generic voxel must be cleared by footprint clearing";
}

// ClearCliffs is the deliberate cliff-wipe path (relocalization, map change).
// It must remove only cliff voxels.
TEST_F(CliffPersistenceTest, ClearCliffsRemovesOnlyCliffVoxels) {
  Mark({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kCliff);
  Mark({MakePoint(0.5, 0.0, 0.1)}, volume_grid::kGeneric);

  grid_->ClearCliffs();

  EXPECT_FALSE(HasVoxelNear(0.0, 0.0))
      << "ClearCliffs must remove cliff voxels";
  EXPECT_TRUE(HasVoxelNear(0.5, 0.0))
      << "ClearCliffs must leave generic voxels untouched";
}

// A later generic observation at a cliff cell must not downgrade its class.
// If it did, footprint clearing would erase the (now generic) voxel.
TEST_F(CliffPersistenceTest, GenericRemarkDoesNotDowngradeCliff) {
  Mark({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kCliff);
  Mark({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kGeneric);

  grid_->ClearCircularArea(0.0, 0.0, 1.0);

  EXPECT_TRUE(HasVoxelNear(0.0, 0.0))
      << "cliff class must persist through a generic re-mark";
}

// The cliff observability cloud must contain only cliff voxels, and its count
// must track marking and ClearCliffs.
TEST_F(CliffPersistenceTest, GetCliffPointCloudReturnsOnlyCliffVoxels) {
  Mark({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kCliff);
  Mark({MakePoint(0.5, 0.0, 0.1)}, volume_grid::kGeneric);

  auto pc2 = boost::make_shared<sensor_msgs::PointCloud2>();
  EXPECT_EQ(grid_->GetCliffPointCloud(pc2), 1u)
      << "cloud must contain only the cliff voxel, not the generic one";
  EXPECT_EQ(pc2->width, 1u);

  grid_->ClearCliffs();
  auto empty_pc2 = boost::make_shared<sensor_msgs::PointCloud2>();
  EXPECT_EQ(grid_->GetCliffPointCloud(empty_pc2), 0u)
      << "no cliff voxels remain after ClearCliffs";
}

// Cliff voxels must never decay. Under LINEAR decay a voxel clears once
// (voxel_decay - time_since_marking) < 0; a negative voxel_decay makes that
// true for every non-cliff voxel on the next decay pass regardless of elapsed
// time, deterministically isolating cliff immunity (generic expires, cliff
// survives).
TEST(CliffDecayTest, CliffSurvivesDecayPassWhileGenericExpires) {
  openvdb::initialize();
  auto grid = std::make_unique<SpatioTemporalVoxelGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, volume_grid::LINEAR,
      /*voxel_decay=*/-1.0, test_utils::kPubVoxels,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(), nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker());

  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, volume_grid::kCliff)});
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.5, 0.0, 0.1)}, volume_grid::kGeneric)});

  RunDecayPass(grid.get());

  EXPECT_TRUE(grid->GetVoxelsAtXY(0.5, 0.0, kQueryTolerance).empty())
      << "generic voxel must expire under decay";
  EXPECT_FALSE(grid->GetVoxelsAtXY(0.0, 0.0, kQueryTolerance).empty())
      << "cliff voxel must survive decay";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_cliff_persistence");
  return RUN_ALL_TESTS();
}
