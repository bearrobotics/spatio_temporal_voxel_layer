/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2026, Bear Robotics, Inc.
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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

#include "bear_msgs/ClassMap.h"
#include "boost/make_shared.hpp"
#include "geometry_msgs/Point.h"
#include "gtest/gtest.h"
#include "openvdb/openvdb.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "spatio_temporal_voxel_layer/class_map_builder.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using test_utils::MaskAt;
using test_utils::TestableGrid;
using volume_grid::ClassMapBuilder;
using volume_grid::ClassMapColumns;
using volume_grid::VoxelClass;
using volume_grid::VoxelClassTable;
using volume_grid::VoxelColumn;

// Long enough that no voxel expires while a test runs.
constexpr double kLongDecaySeconds = 1000.0;
constexpr double kWindowWidthM = 20.0;
constexpr double kSmallWindowWidthM = 4.0;
constexpr double kRobotX = 0.0;
constexpr double kRobotY = 0.0;
constexpr double kVoxelSizeM = test_utils::kVoxelSize;
// Float error of the voxel size, summed over a window.
constexpr double kLatticeTolerance = 1e-6;
// 400 columns across on the lattice, 401 off the lattice.
constexpr std::size_t kOffLatticeWindowWidthColumns = 401;
constexpr double kOffLatticeRobotX = 0.01;
constexpr double kOffLatticeRobotY = 0.01;
// Off the voxel faces, where float error could pick the neighbour.
constexpr double kMarkX = 1.01;
constexpr double kMarkY = 2.01;
constexpr double kMarkZ = 0.51;
constexpr double kLowerZ = 0.41;
constexpr double kUpperZ = 1.01;
constexpr double kAwayX = 5.01;
constexpr double kAwayY = 5.01;
// The layer's voxel size, a double; the 12.3 m window needs the exact value.
constexpr double kCostmapResolutionM = 0.05;
// Float error widens both edges of this 12.3 m window: 248 columns.
constexpr double kWholeVoxelWindowWidthM = 12.3;
constexpr double kEdgeWideningRobotX = -6.45;
constexpr double kEdgeWideningRobotY = -6.45;
constexpr uint32_t kWidenedWindowWidthColumns = 248;

constexpr uint16_t kCliffBit = volume_grid::ToClassBit(VoxelClass::kCliff);
constexpr uint16_t kVehicleBit = volume_grid::ToClassBit(VoxelClass::kVehicle);

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

VoxelColumn ColumnOf(double x, double y) {
  return {static_cast<int32_t>(std::floor(x / kVoxelSizeM)),
          static_cast<int32_t>(std::floor(y / kVoxelSizeM))};
}

observation::MeasurementReading MakeReading(
    const std::vector<geometry_msgs::Point>& points, VoxelClass voxel_class) {
  observation::MeasurementReading reading;
  reading._source_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(points));
  reading._cloud->header.stamp = ros::Time(1000.0);
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  reading._voxel_class = voxel_class;
  return reading;
}

struct ClassMapFlags {
  bool cliff_publishes_map = false;
  bool vehicle_publishes_map = true;
};

VoxelClassTable::Row MakeRow(VoxelClass voxel_class, int priority,
                             bool publish_class_map) {
  VoxelClassTable::Row row;
  row.voxel_class = voxel_class;
  row.policy.decay_seconds = kLongDecaySeconds;
  row.policy.priority = priority;
  row.policy.cleared_by_frustums = false;
  row.policy.cleared_by_dynamic_obstacles = false;
  row.policy.cleared_by_footprint_clear = false;
  row.policy.cleared_by_front_blind_spot = false;
  row.policy.decays_in_inter_sensor_prism = false;
  row.policy.publish_class_voxels = false;
  row.policy.publish_class_map = publish_class_map;
  return row;
}

VoxelClassTable MakeTable(const ClassMapFlags& flags) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = kLongDecaySeconds;
  config.rows.push_back(
      MakeRow(VoxelClass::kCliff, /*priority=*/10, flags.cliff_publishes_map));
  config.rows.push_back(MakeRow(VoxelClass::kVehicle, /*priority=*/1,
                                flags.vehicle_publishes_map));
  return VoxelClassTable::Create(config).value();
}

std::unique_ptr<TestableGrid> MakeGrid(const VoxelClassTable& class_table) {
  openvdb::initialize();
  return std::make_unique<TestableGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, volume_grid::LINEAR,
      /*voxel_decay=*/kLongDecaySeconds, /*pub_voxels=*/false,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(), /*front_blind_spot=*/nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker(),
      std::optional<VoxelClassTable>(class_table));
}

std::unique_ptr<TestableGrid> MakeRecordingGrid(
    const VoxelClassTable& class_table) {
  std::unique_ptr<TestableGrid> grid = MakeGrid(class_table);
  grid->SetClassMapRecording(true);
  return grid;
}

void RunDecayPass(TestableGrid* grid) {
  std::unordered_set<volume_grid::occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  grid->ClearFrustums({}, cleared_cells, dynamic_obstacle_readings,
                      robot_motion_readings);
}

ClassMapColumns TakeColumns(TestableGrid* grid) {
  ClassMapColumns columns;
  grid->TakeClassMap(&columns);
  return columns;
}

bear_msgs::ClassMap BuildMap(TestableGrid* grid,
                             const VoxelClassTable& class_table) {
  ClassMapBuilder builder(kVoxelSizeM, kWindowWidthM, class_table);
  bear_msgs::ClassMap map;
  builder.Fill(TakeColumns(grid), kRobotX, kRobotY, &map);
  return map;
}

std::size_t CountCellsWithBit(const bear_msgs::ClassMap& map, uint16_t bit) {
  return static_cast<std::size_t>(
      std::count_if(map.mask.begin(), map.mask.end(),
                    [bit](uint16_t mask) { return (mask & bit) != 0; }));
}

int32_t ToCostmapLatticeIndex(double lattice_coord) {
  return static_cast<int32_t>(std::lround(lattice_coord / kCostmapResolutionM));
}

void ExpectOnLattice(double world_coord) {
  const double lattice_point =
      std::round(world_coord / kVoxelSizeM) * kVoxelSizeM;
  EXPECT_NEAR(world_coord, lattice_point, kLatticeTolerance);
}

void ExpectWindowAround(const bear_msgs::ClassMap& map, double robot_x,
                        double robot_y, double window_width_m) {
  ExpectOnLattice(map.window_min.x);
  ExpectOnLattice(map.window_min.y);
  ExpectOnLattice(map.window_max.x);
  ExpectOnLattice(map.window_max.y);
  const double half_width = window_width_m / 2.0;
  EXPECT_LE(map.window_min.x, robot_x - half_width + kLatticeTolerance);
  EXPECT_GT(map.window_min.x + kVoxelSizeM, robot_x - half_width);
  EXPECT_LE(map.window_min.y, robot_y - half_width + kLatticeTolerance);
  EXPECT_GT(map.window_min.y + kVoxelSizeM, robot_y - half_width);
  EXPECT_GE(map.window_max.x, robot_x + half_width - kLatticeTolerance);
  EXPECT_LT(map.window_max.x - kVoxelSizeM, robot_x + half_width);
  EXPECT_GE(map.window_max.y, robot_y + half_width - kLatticeTolerance);
  EXPECT_LT(map.window_max.y - kVoxelSizeM, robot_y + half_width);
}

}  // namespace

TEST(ClassMapTest, RecordingOffLeavesNoColumns) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kMarkZ)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  EXPECT_TRUE(TakeColumns(grid.get()).empty());
}

TEST(ClassMapTest, TakeClassMapHandsOverTheColumns) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kMarkZ)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const ClassMapColumns columns = TakeColumns(grid.get());

  ASSERT_EQ(columns.size(), 1u);
  EXPECT_EQ(columns[0].column, ColumnOf(kMarkX, kMarkY));
  EXPECT_EQ(columns[0].mask, kVehicleBit);
  EXPECT_TRUE(TakeColumns(grid.get()).empty());
}

TEST(ClassMapTest, StaleColumnsHandedBackAreDroppedByTheNextPass) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kMarkZ)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());
  ClassMapColumns columns;
  grid->TakeClassMap(&columns);
  ASSERT_EQ(columns.size(), 1u);

  grid->ClearClassVoxels(VoxelClass::kVehicle);
  RunDecayPass(grid.get());
  grid->TakeClassMap(&columns);
  EXPECT_TRUE(columns.empty());

  RunDecayPass(grid.get());
  EXPECT_TRUE(TakeColumns(grid.get()).empty());
}

TEST(ClassMapTest, NoColumnsGivesAnEmptyBoxWithTheWindow) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  EXPECT_EQ(map.info.width, 0u);
  EXPECT_EQ(map.info.height, 0u);
  EXPECT_TRUE(map.mask.empty());
  EXPECT_FLOAT_EQ(map.info.resolution, kVoxelSizeM);
  EXPECT_EQ(map.class_names.size(), volume_grid::kVoxelClassCount);
  ExpectWindowAround(map, kRobotX, kRobotY, kWindowWidthM);
}

TEST(ClassMapTest, NoColumnsEmptiesAReusedMessage) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kMarkZ)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());
  ClassMapBuilder builder(kVoxelSizeM, kWindowWidthM, class_table);
  bear_msgs::ClassMap map;
  builder.Fill(TakeColumns(grid.get()), kRobotX, kRobotY, &map);
  ASSERT_EQ(map.mask.size(), 1u);

  builder.Fill(ClassMapColumns{}, kRobotX, kRobotY, &map);

  EXPECT_EQ(map.info.width, 0u);
  EXPECT_EQ(map.info.height, 0u);
  EXPECT_EQ(map.info.origin.position.x, 0.0);
  EXPECT_EQ(map.info.origin.position.y, 0.0);
  EXPECT_TRUE(map.mask.empty());
}

TEST(ClassMapTest, BoxCoversTheColumnsInsideTheWindowAndNothingElse) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  const geometry_msgs::Point near = MakePoint(kMarkX, kMarkY, kMarkZ);
  const geometry_msgs::Point far =
      MakePoint(kMarkX + 4 * kVoxelSizeM, kMarkY, kMarkZ);
  grid->Mark({MakeReading({near, far}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  ASSERT_EQ(map.mask.size(),
            static_cast<std::size_t>(map.info.width) * map.info.height);
  EXPECT_EQ(map.info.width, 5u);
  EXPECT_EQ(map.info.height, 1u);
  EXPECT_EQ(CountCellsWithBit(map, kVehicleBit), 2u);
  EXPECT_EQ(MaskAt(map, near.x, near.y), kVehicleBit);
  EXPECT_EQ(MaskAt(map, far.x, far.y), kVehicleBit);
  EXPECT_EQ(MaskAt(map, kMarkX + 2 * kVoxelSizeM, kMarkY), 0u);
  EXPECT_FALSE(MaskAt(map, kMarkX, kMarkY + 1.0).has_value());
}

// Cells (0, 0), (2, 0), (0, 1) land at 0, 2, 3 only in row-major order.
TEST(ClassMapTest, MaskIsRowMajor) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  const geometry_msgs::Point corner = MakePoint(kMarkX, kMarkY, kMarkZ);
  const geometry_msgs::Point along_x =
      MakePoint(kMarkX + 2 * kVoxelSizeM, kMarkY, kMarkZ);
  const geometry_msgs::Point along_y =
      MakePoint(kMarkX, kMarkY + kVoxelSizeM, kMarkZ);
  grid->Mark({MakeReading({corner, along_x, along_y}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  ASSERT_EQ(map.info.width, 3u);
  ASSERT_EQ(map.info.height, 2u);
  EXPECT_EQ(map.mask[0], kVehicleBit);
  EXPECT_EQ(map.mask[2], kVehicleBit);
  EXPECT_EQ(map.mask[3], kVehicleBit);
  EXPECT_EQ(CountCellsWithBit(map, kVehicleBit), 3u);
  EXPECT_EQ(MaskAt(map, along_x.x, along_x.y), kVehicleBit);
  EXPECT_EQ(MaskAt(map, along_y.x, along_y.y), kVehicleBit);
}

TEST(ClassMapTest, TwoVoxelsInOneColumnShareACell) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark({MakeReading(
      {MakePoint(kMarkX, kMarkY, kLowerZ), MakePoint(kMarkX, kMarkY, kUpperZ)},
      VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  EXPECT_EQ(map.info.width, 1u);
  EXPECT_EQ(map.info.height, 1u);
  EXPECT_EQ(CountCellsWithBit(map, kVehicleBit), 1u);
}

TEST(ClassMapTest, TwoClassesInOneColumnSetBothBits) {
  ClassMapFlags flags;
  flags.cliff_publishes_map = true;
  const VoxelClassTable class_table = MakeTable(flags);
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kLowerZ)}, VoxelClass::kCliff),
       MakeReading({MakePoint(kMarkX, kMarkY, kUpperZ)},
                   VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kCliffBit | kVehicleBit);
  EXPECT_EQ(map.class_names[static_cast<std::size_t>(VoxelClass::kCliff)],
            "cliff");
  EXPECT_EQ(map.class_names[static_cast<std::size_t>(VoxelClass::kVehicle)],
            "vehicle");
}

TEST(ClassMapTest, RepeatedEntriesOfAColumnShareACell) {
  ClassMapFlags flags;
  flags.cliff_publishes_map = true;
  const VoxelClassTable class_table = MakeTable(flags);
  ClassMapBuilder builder(kVoxelSizeM, kWindowWidthM, class_table);
  const double neighbour_y = kMarkY + kVoxelSizeM;
  const VoxelColumn marked = ColumnOf(kMarkX, kMarkY);
  const VoxelColumn neighbour = ColumnOf(kMarkX, neighbour_y);
  bear_msgs::ClassMap map;

  builder.Fill(
      {{marked, kCliffBit}, {neighbour, kVehicleBit}, {marked, kVehicleBit}},
      kRobotX, kRobotY, &map);

  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kCliffBit | kVehicleBit);
  EXPECT_EQ(MaskAt(map, kMarkX, neighbour_y), kVehicleBit);
}

TEST(ClassMapTest, AClassWithoutTheFlagSetsNoBitAndHasNoName) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kLowerZ)}, VoxelClass::kCliff),
       MakeReading({MakePoint(kMarkX, kMarkY, kUpperZ)},
                   VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kVehicleBit);
  EXPECT_EQ(map.class_names[static_cast<std::size_t>(VoxelClass::kCliff)], "");
  EXPECT_EQ(map.class_names[static_cast<std::size_t>(VoxelClass::kVehicle)],
            "vehicle");
}

TEST(ClassMapTest, GenericVoxelsSetNoBit) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kLowerZ)}, VoxelClass::kGeneric),
       MakeReading({MakePoint(kMarkX, kMarkY, kUpperZ)}, VoxelClass::kVehicle),
       MakeReading({MakePoint(kAwayX, kAwayY, kMarkZ)}, VoxelClass::kGeneric)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kVehicleBit);
  EXPECT_EQ(map.class_names[static_cast<std::size_t>(VoxelClass::kGeneric)],
            "");
  EXPECT_EQ(map.info.width, 1u);
  EXPECT_FALSE(MaskAt(map, kAwayX, kAwayY).has_value());
}

TEST(ClassMapTest, ColumnsOutsideTheWindowAreLeftOut) {
  const VoxelClassTable class_table = MakeTable({});
  const geometry_msgs::Point near = MakePoint(kMarkX, kMarkY, kMarkZ);
  const geometry_msgs::Point far =
      MakePoint(kMarkX + 1.5 * kWindowWidthM, kMarkY, kMarkZ);
  const geometry_msgs::Point far_in_y =
      MakePoint(kMarkX, kMarkY + 1.5 * kWindowWidthM, kMarkZ);
  ClassMapBuilder builder(kVoxelSizeM, kWindowWidthM, class_table);

  auto grid = MakeRecordingGrid(class_table);
  grid->Mark({MakeReading({near, far, far_in_y}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());
  const ClassMapColumns columns = TakeColumns(grid.get());

  bear_msgs::ClassMap around_origin;
  builder.Fill(columns, kRobotX, kRobotY, &around_origin);
  EXPECT_EQ(around_origin.info.width, 1u);
  EXPECT_EQ(around_origin.info.height, 1u);
  EXPECT_EQ(MaskAt(around_origin, near.x, near.y), kVehicleBit);
  EXPECT_FALSE(MaskAt(around_origin, far.x, far.y).has_value());
  EXPECT_FALSE(MaskAt(around_origin, far_in_y.x, far_in_y.y).has_value());

  bear_msgs::ClassMap around_far;
  builder.Fill(columns, far.x, far.y, &around_far);
  EXPECT_EQ(around_far.info.width, 1u);
  EXPECT_EQ(MaskAt(around_far, far.x, far.y), kVehicleBit);
  EXPECT_FALSE(MaskAt(around_far, near.x, near.y).has_value());
  ExpectWindowAround(around_far, far.x, far.y, kWindowWidthM);
}

TEST(ClassMapTest, WindowFollowsTheConfiguredSize) {
  const VoxelClassTable class_table = MakeTable({});
  ClassMapBuilder builder(kVoxelSizeM, kSmallWindowWidthM, class_table);
  const double beyond_small_window_x = kMarkX + kSmallWindowWidthM;
  bear_msgs::ClassMap map;

  builder.Fill({{ColumnOf(kMarkX, kMarkY), kVehicleBit},
                {ColumnOf(beyond_small_window_x, kMarkY), kVehicleBit}},
               kMarkX, kMarkY, &map);

  ExpectWindowAround(map, kMarkX, kMarkY, kSmallWindowWidthM);
  EXPECT_EQ(map.info.width, 1u);
  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kVehicleBit);
  EXPECT_FALSE(MaskAt(map, beyond_small_window_x, kMarkY).has_value());
}

TEST(ClassMapTest, ReservedMaskHoldsABoxSpanningTheWindow) {
  const VoxelClassTable class_table = MakeTable({});
  ClassMapBuilder builder(kCostmapResolutionM, kWholeVoxelWindowWidthM,
                          class_table);
  bear_msgs::ClassMap map;
  builder.Fill(ClassMapColumns{}, kEdgeWideningRobotX, kEdgeWideningRobotY,
               &map);
  const std::size_t reserved = map.mask.capacity();
  const VoxelColumn lower_corner{ToCostmapLatticeIndex(map.window_min.x),
                                 ToCostmapLatticeIndex(map.window_min.y)};
  const VoxelColumn upper_corner{ToCostmapLatticeIndex(map.window_max.x) - 1,
                                 ToCostmapLatticeIndex(map.window_max.y) - 1};

  builder.Fill({{lower_corner, kVehicleBit}, {upper_corner, kVehicleBit}},
               kEdgeWideningRobotX, kEdgeWideningRobotY, &map);

  ASSERT_EQ(map.info.width, kWidenedWindowWidthColumns);
  ASSERT_EQ(map.info.height, kWidenedWindowWidthColumns);
  EXPECT_GE(reserved, map.mask.size());
  EXPECT_EQ(map.mask.capacity(), reserved);
}

TEST(ClassMapTest, OriginLiesOnTheVoxelLattice) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(1.03, 2.07, 0.51)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());

  const bear_msgs::ClassMap map = BuildMap(grid.get(), class_table);

  // The voxel holding (1.03, 2.07) spans [1.00, 1.05) x [2.05, 2.10).
  EXPECT_NEAR(map.info.origin.position.x, 1.00, kLatticeTolerance);
  EXPECT_NEAR(map.info.origin.position.y, 2.05, kLatticeTolerance);
  EXPECT_EQ(MaskAt(map, 1.049, 2.099), kVehicleBit);
  EXPECT_FALSE(MaskAt(map, 1.051, 2.07).has_value());
  EXPECT_FALSE(MaskAt(map, 1.03, 2.101).has_value());
}

TEST(ClassMapTest, MaskIsReservedForTheWindowOnceAndKept) {
  const VoxelClassTable class_table = MakeTable({});
  auto grid = MakeRecordingGrid(class_table);
  grid->Mark(
      {MakeReading({MakePoint(kMarkX, kMarkY, kMarkZ)}, VoxelClass::kVehicle)});
  RunDecayPass(grid.get());
  const ClassMapColumns columns = TakeColumns(grid.get());
  ClassMapBuilder builder(kVoxelSizeM, kWindowWidthM, class_table);

  bear_msgs::ClassMap map;
  builder.Fill(columns, kRobotX, kRobotY, &map);
  const std::size_t reserved = map.mask.capacity();
  EXPECT_GE(reserved,
            kOffLatticeWindowWidthColumns * kOffLatticeWindowWidthColumns);

  builder.Fill(columns, kOffLatticeRobotX, kOffLatticeRobotY, &map);
  EXPECT_EQ(map.mask.capacity(), reserved);
  ExpectWindowAround(map, kOffLatticeRobotX, kOffLatticeRobotY, kWindowWidthM);
  EXPECT_EQ(MaskAt(map, kMarkX, kMarkY), kVehicleBit);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_class_map");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
