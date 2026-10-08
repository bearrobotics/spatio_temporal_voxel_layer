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
#include <geometry_msgs/Quaternion.h>
#include <openvdb/openvdb.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using volume_grid::SpatioTemporalVoxelGrid;
using volume_grid::VoxelClass;
using volume_grid::VoxelClassPolicy;
using volume_grid::VoxelClassTable;

constexpr double kQueryTolerance = 0.07;
constexpr double kDecayExpired = -1.0;
constexpr double kDecayNever = 1000.0;
// factor * age^3 / 6 with these values is 10000 s, far past kDecayNever.
constexpr double kAgedMarkSeconds = 10.0;
constexpr double kFrustumAcceleration = 60.0;
constexpr float kOtherRobotRadius = 0.3f;
constexpr double kClearingInflation = 1.2;

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

// The camera at the origin looks along +z, so these sit inside its frustum.
const geometry_msgs::Point kCliffInFrustum = MakePoint(0.3, 0.0, 1.0);
const geometry_msgs::Point kGenericInFrustum = MakePoint(0.3, 0.3, 1.0);
// Both sit inside the clearing circles of a robot driving (0,0) to (1,0).
const geometry_msgs::Point kCliffOnRobotPath = MakePoint(0.5, 0.0, 0.1);
const geometry_msgs::Point kGenericOnRobotPath = MakePoint(0.5, 0.2, 0.1);

observation::MeasurementReading MakeMarkingReading(
    const std::vector<geometry_msgs::Point>& points, VoxelClass voxel_class) {
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

observation::MeasurementReading MakeGenericClearingReading() {
  ros::NodeHandle frustum_nh("/class_persistence_test/frustum");
  observation::MeasurementReading reading;
  reading._sensor_name = "camera";
  reading._cloud->header.stamp = ros::Time(1.0);
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._orientation = MakeIdentityQuaternion();
  reading._frustrum_factory =
      FrustumFactoryFactory::CreateFrustumFactory(frustum_nh);
  reading._marking = false;
  reading._clearing = true;
  reading._obstacle_range_in_m = 100.0;
  reading._decay_acceleration = kFrustumAcceleration;
  return reading;
}

std::optional<VoxelClassTable> MakeCliffTable(
    double generic_decay, double cliff_decay,
    bool cliff_cleared_by_footprint = false) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = generic_decay;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = cliff_decay;
  row.policy.priority = 10;
  row.policy.cleared_by_frustums = true;
  row.policy.cleared_by_dynamic_obstacles = false;
  row.policy.cleared_by_footprint_clear = cliff_cleared_by_footprint;
  row.policy.cleared_by_front_blind_spot = false;
  row.policy.decays_in_inter_sensor_prism = false;
  config.rows.push_back(row);
  return VoxelClassTable::Create(config);
}

class TestableGrid : public SpatioTemporalVoxelGrid {
 public:
  using SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid;

  // Not Grid::empty(), which stays false until a cleared voxel is pruned.
  [[nodiscard]] bool IsClassGridEmpty() const {
    return _class_grid->activeVoxelCount() == 0;
  }

  // The cloud stores float, so round the same way before indexing.
  [[nodiscard]] openvdb::Coord CoordAt(
      const geometry_msgs::Point& point) const {
    const openvdb::Vec3d index =
        WorldToIndex({static_cast<float>(point.x), static_cast<float>(point.y),
                      static_cast<float>(point.z)});
    return openvdb::Coord(index[0], index[1], index[2]);
  }

  [[nodiscard]] VoxelClass ClassAt(const geometry_msgs::Point& point) const {
    return volume_grid::ToVoxelClass(
        _class_grid->getAccessor().getValue(CoordAt(point)));
  }

  // Rewinds a mark time so a pass sees an old mark without waiting.
  void AgeMark(const geometry_msgs::Point& point, double seconds) {
    const openvdb::Coord coord = CoordAt(point);
    openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();
    accessor.setValueOn(coord, accessor.getValue(coord) - seconds);
  }

  [[nodiscard]] bool IsClassGridConsistent() const {
    openvdb::DoubleGrid::ConstAccessor values = _grid->getConstAccessor();
    for (openvdb::Int32Grid::ValueOnCIter cit = _class_grid->cbeginValueOn();
         cit.test(); ++cit) {
      if (cit.getValue() == volume_grid::ToClassId(VoxelClass::kGeneric) ||
          !values.isValueOn(cit.getCoord())) {
        return false;
      }
    }
    return true;
  }
};

std::unique_ptr<TestableGrid> MakeGrid(
    std::optional<VoxelClassTable> table, int decay_model,
    std::unique_ptr<geometry::FootprintClearingPrism> front_blind_spot =
        nullptr,
    std::unique_ptr<RobotMotionTracker> robot_motion_tracker = nullptr) {
  openvdb::initialize();
  if (!robot_motion_tracker) {
    robot_motion_tracker = test_utils::MakeMockRobotMotionTracker();
  }
  // voxel_decay only seeds a table the caller did not supply, so it is unused.
  return std::make_unique<TestableGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, decay_model,
      /*voxel_decay=*/0.0, /*pub_voxels=*/false,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(), std::move(front_blind_spot),
      test_utils::MakeMockDynamicObstacleTracker(),
      std::move(robot_motion_tracker), std::move(table));
}

std::unique_ptr<RobotMotionTracker> MakeEnabledRobotMotionTracker() {
  ros::NodeHandle nh;
  RobotMotionTracker::Config config;
  config.enable = true;
  config.activation_velocity_threshold = 0.1;
  config.min_distance_between_readings_threshold = 0.25;
  config.stale_time_threshold = 2.0;
  config.inflation_radius_factor = kClearingInflation;
  config.number_of_interpolation_circles = 3;
  config.past_time_window = 1.0;
  config.publish_visualization = false;
  return RobotMotionTracker::Create(config, nh);
}

RobotMotionReading MakeOtherRobotReading(double x, double y) {
  RobotMotionReading reading;
  reading.time_ = ros::Time::now();
  reading.robot_id_ = "other_robot";
  reading.center_ = Eigen::Vector2d(x, y);
  reading.velocity_ = Eigen::Vector2d(1.0, 0.0);
  reading.radius_ = kOtherRobotRadius;
  return reading;
}

std::unique_ptr<geometry::FootprintClearingPrism> MakeFrontBlindSpotPrism() {
  ros::NodeHandle nh;
  geometry::FootprintClearingPrism::Config config;
  config.enable = true;
  config.publish_visualization = false;
  config.min_z = 0.0;
  config.max_z = 1.0;
  config.footprint_points = {
      {-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  return geometry::FootprintClearingPrism::Create(config, nh);
}

void RunDecayPass(
    SpatioTemporalVoxelGrid* grid,
    const std::vector<observation::MeasurementReading>& clearing_readings = {},
    std::vector<RobotMotionReading> robot_motion_readings = {}) {
  std::unordered_set<volume_grid::occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  grid->ClearFrustums(clearing_readings, cleared_cells,
                      dynamic_obstacle_readings, robot_motion_readings);
}

bool HasVoxelNear(SpatioTemporalVoxelGrid* grid, double x, double y) {
  return !grid->GetVoxelsAtXY(x, y, kQueryTolerance).empty();
}

bool HasVoxelNear(SpatioTemporalVoxelGrid* grid,
                  const geometry_msgs::Point& point) {
  return HasVoxelNear(grid, point.x, point.y);
}

TEST(VoxelClassTableTest, ValidCliffConfigBuilds) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = 15.0;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = 120.0;
  row.policy.priority = 10;
  config.rows.push_back(row);

  const std::optional<VoxelClassTable> table = VoxelClassTable::Create(config);
  ASSERT_TRUE(table.has_value());
  EXPECT_EQ(table->GetPolicy(VoxelClass::kCliff).decay_seconds, 120.0);
  EXPECT_EQ(table->GetPolicy(VoxelClass::kCliff).priority, 10);
  EXPECT_EQ(table->GetPolicy(VoxelClass::kGeneric).decay_seconds, 15.0);
  EXPECT_EQ(table->GetPolicy(VoxelClass::kGeneric).priority, 0);
}

TEST(VoxelClassTableTest, ConfiguringGenericRejected) {
  VoxelClassTable::Config config;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kGeneric;
  row.policy.decay_seconds = 120.0;
  row.policy.priority = 5;
  config.rows.push_back(row);
  EXPECT_FALSE(VoxelClassTable::Create(config).has_value());
}

TEST(VoxelClassTableTest, PriorityBelowOneRejected) {
  VoxelClassTable::Config config;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = 120.0;
  row.policy.priority = 0;
  config.rows.push_back(row);
  EXPECT_FALSE(VoxelClassTable::Create(config).has_value());
}

TEST(VoxelClassTableTest, NonPositiveDecayRejected) {
  VoxelClassTable::Config config;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = 0.0;
  row.policy.priority = 10;
  config.rows.push_back(row);
  EXPECT_FALSE(VoxelClassTable::Create(config).has_value());
}

TEST(VoxelClassTableTest, InfiniteDecayRejected) {
  VoxelClassTable::Config config;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = std::numeric_limits<double>::infinity();
  row.policy.priority = 10;
  config.rows.push_back(row);
  EXPECT_FALSE(VoxelClassTable::Create(config).has_value());
}

TEST(VoxelClassTableTest, DuplicateClassRejected) {
  VoxelClassTable::Config config;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = 120.0;
  row.policy.priority = 10;
  config.rows.push_back(row);
  config.rows.push_back(row);
  EXPECT_FALSE(VoxelClassTable::Create(config).has_value());
}

TEST(VoxelClassTableTest, IsConfiguredTracksConfiguredClasses) {
  const std::optional<VoxelClassTable> table = MakeCliffTable(15.0, 120.0);
  ASSERT_TRUE(table.has_value());
  EXPECT_TRUE(table->IsConfigured(VoxelClass::kGeneric));
  EXPECT_TRUE(table->IsConfigured(VoxelClass::kCliff));

  const VoxelClassTable generic_only = VoxelClassTable::CreateGenericOnly(15.0);
  EXPECT_TRUE(generic_only.IsConfigured(VoxelClass::kGeneric));
  EXPECT_FALSE(generic_only.IsConfigured(VoxelClass::kCliff));
}

TEST(VoxelClassTableTest, ParseVoxelClassMapsKnownNames) {
  EXPECT_EQ(volume_grid::ParseVoxelClass("generic"), VoxelClass::kGeneric);
  EXPECT_EQ(volume_grid::ParseVoxelClass("cliff"), VoxelClass::kCliff);
  EXPECT_FALSE(volume_grid::ParseVoxelClass("vehicle").has_value());
}

TEST(VoxelClassTableTest, OutOfRangeClassIdReadsAsGeneric) {
  // The corrupt-id log is throttled, which reads ros::Time::now().
  ros::NodeHandle nh;
  constexpr int32_t kBeyondLastClass =
      static_cast<int32_t>(volume_grid::kVoxelClassCount);
  EXPECT_EQ(volume_grid::ToVoxelClass(kBeyondLastClass), VoxelClass::kGeneric);
  EXPECT_EQ(volume_grid::ToVoxelClass(-1), VoxelClass::kGeneric);

  const std::optional<VoxelClassTable> table = MakeCliffTable(15.0, 120.0);
  ASSERT_TRUE(table.has_value());
  const VoxelClassPolicy& policy =
      table->GetPolicy(static_cast<VoxelClass>(kBeyondLastClass));
  EXPECT_EQ(policy.decay_seconds, 15.0);
  EXPECT_EQ(policy.priority, 0);
}

TEST(VoxelClassGridTest, PerClassDecayDifferentiation) {
  auto grid = MakeGrid(MakeCliffTable(/*generic_decay=*/kDecayExpired,
                                      /*cliff_decay=*/kDecayNever),
                       volume_grid::LINEAR);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.5, 0.0, 0.1)}, VoxelClass::kGeneric)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.5, 0.0));

  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "cliff (kDecayNever) must survive the pass";
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.5, 0.0))
      << "generic (kDecayExpired) must expire in the pass";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, PolicyClearedCliffLeavesNoClassEntry) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever, kDecayNever,
                                      /*cliff_cleared_by_footprint=*/true),
                       volume_grid::LINEAR);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));

  grid->ClearCircularArea(0.0, 0.0, 1.0);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a cliff that opts into footprint clearing must be removed";
  EXPECT_TRUE(grid->IsClassGridConsistent())
      << "clearing a cliff must clear its class-grid entry too";
}

TEST(VoxelClassGridTest, GenericRemarkDoesNotDowngradeCliff) {
  const geometry_msgs::Point mark = MakePoint(0.0, 0.0, 0.1);
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->Mark({MakeMarkingReading({mark}, VoxelClass::kCliff)});

  grid->Mark({MakeMarkingReading({mark}, VoxelClass::kGeneric)});

  EXPECT_EQ(grid->ClassAt(mark), VoxelClass::kCliff);
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, CliffMarkPromotesGenericVoxel) {
  const geometry_msgs::Point mark = MakePoint(0.0, 0.0, 0.1);
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->Mark({MakeMarkingReading({mark}, VoxelClass::kGeneric)});
  ASSERT_EQ(grid->ClassAt(mark), VoxelClass::kGeneric);

  grid->Mark({MakeMarkingReading({mark}, VoxelClass::kCliff)});

  EXPECT_EQ(grid->ClassAt(mark), VoxelClass::kCliff);
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, FootprintClearPreservesCliffClearsGeneric) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.5, 0.0, 0.1)}, VoxelClass::kGeneric)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.5, 0.0));

  grid->ClearCircularArea(0.25, 0.0, 1.0);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "cliff must survive footprint clearing";
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.5, 0.0))
      << "generic must be cleared by footprint clearing";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, FrontBlindSpotPreservesCliffClearsGeneric) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever, kDecayNever),
                       volume_grid::LINEAR, MakeFrontBlindSpotPrism());
  grid->SetRobotPose(0.0, 0.0, 0.0);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.5, 0.0, 0.1)}, VoxelClass::kGeneric)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.5, 0.0));

  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "cliff must survive the front blind-spot prism";
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.5, 0.0))
      << "generic must be cleared by the front blind-spot prism";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, ResetGridAreaClearsEveryClass) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));

  grid->ResetGridArea(volume_grid::occupany_cell(-1.0, -1.0),
                      volume_grid::occupany_cell(1.0, 1.0),
                      /*invert_area=*/true);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_TRUE(grid->IsClassGridEmpty());
}

TEST(VoxelClassGridTest, ResetGridClearsEveryClass) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));

  EXPECT_TRUE(grid->ResetGrid());

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_TRUE(grid->IsClassGridEmpty());
}

TEST(VoxelClassGridTest, GenericOnlyKeepsClassGridEmpty) {
  auto grid = MakeGrid(std::nullopt, volume_grid::LINEAR);

  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kGeneric)});

  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_TRUE(grid->IsClassGridEmpty());
}

TEST(VoxelClassGridTest, NonMarkingReadingLeavesGridUntouched) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  observation::MeasurementReading reading =
      MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff);
  reading._marking = false;

  grid->Mark({reading});

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_TRUE(grid->IsClassGridEmpty());
}

TEST(VoxelClassGridTest, ExponentialModelKeepsFreshMarks) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever, kDecayNever),
                       volume_grid::EXPONENTIAL);
  grid->SetRobotPose(0.0, 0.0, 0.0);
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.0, 0.0, 0.1)}, VoxelClass::kCliff)});
  grid->Mark(
      {MakeMarkingReading({MakePoint(0.5, 0.0, 0.1)}, VoxelClass::kGeneric)});

  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.5, 0.0));
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, GenericFrustumDoesNotClearCliff) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR);
  grid->SetRobotPose(0.0, 0.0, 0.0);
  grid->Mark({MakeMarkingReading({kCliffInFrustum}, VoxelClass::kCliff)});
  grid->Mark({MakeMarkingReading({kGenericInFrustum}, VoxelClass::kGeneric)});
  grid->AgeMark(kCliffInFrustum, kAgedMarkSeconds);
  grid->AgeMark(kGenericInFrustum, kAgedMarkSeconds);
  ASSERT_TRUE(HasVoxelNear(grid.get(), kCliffInFrustum));
  ASSERT_TRUE(HasVoxelNear(grid.get(), kGenericInFrustum));

  RunDecayPass(grid.get(), {MakeGenericClearingReading()});

  EXPECT_FALSE(HasVoxelNear(grid.get(), kGenericInFrustum))
      << "an aged generic mark inside a generic frustum must be cleared";
  EXPECT_TRUE(HasVoxelNear(grid.get(), kCliffInFrustum))
      << "a generic frustum must not clear a cliff";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(VoxelClassGridTest, OtherRobotPathClearsGenericButNotCliff) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, kDecayNever), volume_grid::LINEAR,
               /*front_blind_spot=*/nullptr, MakeEnabledRobotMotionTracker());
  grid->SetRobotPose(0.0, 0.0, 0.0);
  grid->Mark({MakeMarkingReading({kCliffOnRobotPath}, VoxelClass::kCliff)});
  grid->Mark({MakeMarkingReading({kGenericOnRobotPath}, VoxelClass::kGeneric)});
  ASSERT_TRUE(HasVoxelNear(grid.get(), kCliffOnRobotPath));
  ASSERT_TRUE(HasVoxelNear(grid.get(), kGenericOnRobotPath));

  // The tracker draws clearing circles only between two readings of a robot.
  RunDecayPass(grid.get(), {}, {MakeOtherRobotReading(0.0, 0.0)});
  RunDecayPass(grid.get(), {}, {MakeOtherRobotReading(1.0, 0.0)});

  EXPECT_FALSE(HasVoxelNear(grid.get(), kGenericOnRobotPath))
      << "a generic mark under another robot's path must be cleared";
  EXPECT_TRUE(HasVoxelNear(grid.get(), kCliffOnRobotPath))
      << "a cliff that opts out of dynamic obstacle clearing must survive";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_class_persistence");
  return RUN_ALL_TESTS();
}
