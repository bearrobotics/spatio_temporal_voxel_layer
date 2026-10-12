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

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/Point.h"
#include "gtest/gtest.h"
#include "openvdb/io/File.h"
#include "openvdb/openvdb.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using test_utils::TestableGrid;
using volume_grid::VoxelClass;
using volume_grid::VoxelClassTable;

constexpr double kQueryTolerance = 0.07;
// Long enough that no voxel expires while a test runs.
constexpr double kLongDecaySeconds = 1000.0;

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
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

struct CliffOptions {
  bool publish_class_voxels = true;
  int confirmation_frames = 1;
  double confirmation_window_seconds = 0.0;
};

std::optional<VoxelClassTable> MakeCliffTable(const CliffOptions& options) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = kLongDecaySeconds;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = kLongDecaySeconds;
  row.policy.priority = 10;
  row.policy.confirmation_frames = options.confirmation_frames;
  row.policy.confirmation_window_seconds = options.confirmation_window_seconds;
  row.policy.cleared_by_frustums = false;
  row.policy.cleared_by_dynamic_obstacles = false;
  row.policy.cleared_by_footprint_clear = false;
  row.policy.cleared_by_front_blind_spot = false;
  row.policy.decays_in_inter_sensor_prism = false;
  row.policy.publish_class_voxels = options.publish_class_voxels;
  config.rows.push_back(row);
  return VoxelClassTable::Create(config);
}

std::optional<VoxelClassTable> MakePublishingCliffTable() {
  return MakeCliffTable(CliffOptions());
}

std::unique_ptr<TestableGrid> MakeGrid(
    std::optional<VoxelClassTable> class_table,
    bool publish_all_voxels = false) {
  openvdb::initialize();
  return std::make_unique<TestableGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, volume_grid::LINEAR,
      /*voxel_decay=*/kLongDecaySeconds, /*pub_voxels=*/publish_all_voxels,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(), /*front_blind_spot=*/nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker(), std::move(class_table));
}

void RunDecayPass(TestableGrid* grid) {
  std::unordered_set<volume_grid::occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  grid->ClearFrustums({}, cleared_cells, dynamic_obstacle_readings,
                      robot_motion_readings);
}

std::vector<geometry_msgs::Point> ReadCloudPoints(
    const sensor_msgs::PointCloud2& cloud) {
  std::vector<geometry_msgs::Point> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back(MakePoint(*iter_x, *iter_y, *iter_z));
  }
  return points;
}

std::vector<geometry_msgs::Point> GetClassCloudPoints(TestableGrid* grid,
                                                      VoxelClass voxel_class) {
  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
  grid->GetClassPointCloud(voxel_class, cloud);
  return ReadCloudPoints(*cloud);
}

std::vector<geometry_msgs::Point> GetAllVoxelsCloudPoints(TestableGrid* grid) {
  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
  grid->GetAllVoxelsPointCloud(cloud);
  return ReadCloudPoints(*cloud);
}

std::string TempFilePath(const std::string& file_name) {
  const char* temp_dir = std::getenv("TEST_TMPDIR");
  return std::string(temp_dir != nullptr ? temp_dir : "/tmp") + "/" + file_name;
}

bool HasVoxelNear(TestableGrid* grid, const geometry_msgs::Point& point) {
  return !grid->GetVoxelsAtXY(point.x, point.y, kQueryTolerance).empty();
}

std::size_t GetFlattenedCostmapCellCount(TestableGrid* grid) {
  return grid->GetFlattenedCostmap()->size();
}

geometry_msgs::Point CliffVoxelPoint() { return MakePoint(0.0, 0.0, 0.1); }
geometry_msgs::Point GenericVoxelPoint() { return MakePoint(1.0, 1.0, 0.1); }

std::unique_ptr<TestableGrid> MakeGridWithMarkedCliff() {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});
  RunDecayPass(grid.get());
  return grid;
}

TEST(ClassCloudTest, ClassCloudFillsWhileWholeGridPublishingIsOff) {
  auto grid = MakeGridWithMarkedCliff();

  EXPECT_EQ(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).size(), 1u)
      << "a class cloud and the all voxels cloud must switch separately";
}

TEST(ClassCloudTest, ClassCloudPointMatchesTheMarkedVoxel) {
  auto grid = MakeGridWithMarkedCliff();

  const std::vector<geometry_msgs::Point> points =
      GetClassCloudPoints(grid.get(), VoxelClass::kCliff);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, CliffVoxelPoint().x, test_utils::kVoxelSize);
  EXPECT_NEAR(points[0].y, CliffVoxelPoint().y, test_utils::kVoxelSize);
  EXPECT_NEAR(points[0].z, CliffVoxelPoint().z, test_utils::kVoxelSize);
}

TEST(ClassCloudTest, ClassCloudIsEmptyBeforeTheFirstDecayPass) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});

  EXPECT_TRUE(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).empty())
      << "Mark must not fill the class cloud without a decay pass";
}

TEST(ClassCloudTest, ClassWithPublishingOffGetsNoPoints) {
  CliffOptions options;
  options.publish_class_voxels = false;
  auto grid = MakeGrid(MakeCliffTable(options));
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});
  RunDecayPass(grid.get());

  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_TRUE(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).empty());
}

TEST(ClassCloudTest, ClassWithPublishingOffStaysInTheAllVoxelsCloud) {
  CliffOptions options;
  options.publish_class_voxels = false;
  auto grid = MakeGrid(MakeCliffTable(options), /*publish_all_voxels=*/true);
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});
  RunDecayPass(grid.get());

  EXPECT_EQ(GetAllVoxelsCloudPoints(grid.get()).size(), 1u)
      << "publish_class_voxels must pick a cloud, never drop a voxel";
  EXPECT_TRUE(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).empty());
}

TEST(ClassCloudTest, MarkedCliffEntersTheAllVoxelsAndClassClouds) {
  auto grid = MakeGrid(MakePublishingCliffTable(), /*publish_all_voxels=*/true);
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});
  RunDecayPass(grid.get());

  EXPECT_EQ(GetAllVoxelsCloudPoints(grid.get()).size(), 1u);
  EXPECT_EQ(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).size(), 1u);
}

TEST(ClassCloudTest, GenericVoxelStaysOutOfTheClassCloud) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({GenericVoxelPoint()}, VoxelClass::kGeneric)});
  RunDecayPass(grid.get());

  ASSERT_TRUE(HasVoxelNear(grid.get(), GenericVoxelPoint()));
  EXPECT_TRUE(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).empty());
}

TEST(ClassCloudTest, ClassCloudHoldsEveryMarkedVoxel) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({MakePoint(0.0, 0.0, 0.1), MakePoint(0.5, 0.0, 0.1),
                           MakePoint(1.0, 0.0, 0.1)},
                          VoxelClass::kCliff)});
  RunDecayPass(grid.get());

  EXPECT_EQ(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).size(), 3u);
}

TEST(ClassCloudTest, ClassCloudDoesNotAccumulateAcrossPasses) {
  auto grid = MakeGridWithMarkedCliff();
  RunDecayPass(grid.get());

  EXPECT_EQ(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).size(), 1u)
      << "each decay pass must rewrite the class cloud from an empty buffer";
}

TEST(ClassCloudTest, ClassCloudDropsAClearedVoxel) {
  auto grid = MakeGridWithMarkedCliff();
  ASSERT_EQ(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).size(), 1u);

  grid->ClearClassVoxels(VoxelClass::kCliff);
  RunDecayPass(grid.get());

  EXPECT_TRUE(GetClassCloudPoints(grid.get(), VoxelClass::kCliff).empty());
}

TEST(ClassClearTest, ClearClassVoxelsRemovesTheMarkedVoxel) {
  auto grid = MakeGridWithMarkedCliff();
  ASSERT_TRUE(HasVoxelNear(grid.get(), CliffVoxelPoint()));

  grid->ClearClassVoxels(VoxelClass::kCliff);

  EXPECT_FALSE(HasVoxelNear(grid.get(), CliffVoxelPoint()));
  EXPECT_TRUE(grid->IsClassGridConsistent())
      << "a class entry must disappear with the voxel";
}

TEST(ClassClearTest, ClearClassVoxelsKeepsGenericVoxels) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff),
              MakeReading({GenericVoxelPoint()}, VoxelClass::kGeneric)});

  grid->ClearClassVoxels(VoxelClass::kCliff);

  EXPECT_FALSE(HasVoxelNear(grid.get(), CliffVoxelPoint()));
  EXPECT_TRUE(HasVoxelNear(grid.get(), GenericVoxelPoint()));
}

TEST(ClassClearTest, ClearClassVoxelsReturnsTheRemovedCount) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({MakePoint(0.0, 0.0, 0.1), MakePoint(0.5, 0.0, 0.1),
                           MakePoint(1.0, 0.0, 0.1)},
                          VoxelClass::kCliff)});

  EXPECT_EQ(grid->ClearClassVoxels(VoxelClass::kCliff), 3u);
}

TEST(ClassClearTest, ClearClassVoxelsEmptiesTheClassGrid) {
  auto grid = MakeGridWithMarkedCliff();
  ASSERT_FALSE(grid->IsClassGridEmpty());

  grid->ClearClassVoxels(VoxelClass::kCliff);

  EXPECT_TRUE(grid->IsClassGridEmpty());
}

TEST(ClassClearTest, ClearClassVoxelsIgnoresGeneric) {
  auto grid = MakeGrid(MakePublishingCliffTable());
  grid->Mark({MakeReading({GenericVoxelPoint()}, VoxelClass::kGeneric)});

  EXPECT_EQ(grid->ClearClassVoxels(VoxelClass::kGeneric), 0u);
  EXPECT_TRUE(HasVoxelNear(grid.get(), GenericVoxelPoint()))
      << "a clear must never remove a voxel that carries no class";
}

TEST(ClassClearTest, ClearClassVoxelsDropsPendingPromotionEvidence) {
  CliffOptions options;
  options.confirmation_frames = 3;
  options.confirmation_window_seconds = 1.0;
  auto grid = MakeGrid(MakeCliffTable(options));
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff)});
  ASSERT_EQ(grid->GetPendingPromotionCount(), 1u);

  grid->ClearClassVoxels(VoxelClass::kCliff);

  EXPECT_EQ(grid->GetPendingPromotionCount(), 0u)
      << "a cleared voxel must not return from evidence left behind";
}

TEST(ClassClearTest, FlattenedCostmapDropsAClearedVoxel) {
  auto grid = MakeGridWithMarkedCliff();
  ASSERT_EQ(GetFlattenedCostmapCellCount(grid.get()), 1u);

  grid->ClearClassVoxels(VoxelClass::kCliff);
  RunDecayPass(grid.get());

  EXPECT_EQ(GetFlattenedCostmapCellCount(grid.get()), 0u)
      << "a cleared voxel must stop marking its costmap cell";
}

TEST(ClassSaveTest, SavedFileCarriesTheValueAndClassGrids) {
  auto grid = MakeGridWithMarkedCliff();
  const std::string file_name = TempFilePath("stvl_save_test");

  double map_size_bytes = 0.0;
  ASSERT_TRUE(grid->SaveGrid(file_name, map_size_bytes));

  openvdb::io::File saved_file(file_name + ".vdb");
  saved_file.open();
  std::vector<std::string> grid_names;
  for (openvdb::io::File::NameIterator it = saved_file.beginName();
       it != saved_file.endName(); ++it) {
    grid_names.push_back(it.gridName());
  }
  saved_file.close();

  EXPECT_NE(std::find(grid_names.begin(), grid_names.end(),
                      "SpatioTemporalVoxelLayer"),
            grid_names.end());
  EXPECT_NE(std::find(grid_names.begin(), grid_names.end(),
                      "SpatioTemporalVoxelClass"),
            grid_names.end())
      << "a save must write the class ids, not only the voxel values";
}

TEST(ClassSaveTest, SavedClassGridKeepsTheCliffMark) {
  auto grid = MakeGridWithMarkedCliff();
  const std::string file_name = TempFilePath("stvl_class_roundtrip_test");
  double map_size_bytes = 0.0;
  ASSERT_TRUE(grid->SaveGrid(file_name, map_size_bytes));

  openvdb::io::File saved_file(file_name + ".vdb");
  saved_file.open();
  const openvdb::GridBase::Ptr read_grid =
      saved_file.readGrid("SpatioTemporalVoxelClass");
  saved_file.close();
  const openvdb::Int32Grid::Ptr class_grid =
      openvdb::gridPtrCast<openvdb::Int32Grid>(read_grid);
  ASSERT_TRUE(class_grid);

  openvdb::Int32Grid::ValueOnCIter cit = class_grid->cbeginValueOn();
  ASSERT_TRUE(cit.test())
      << "a save must keep the class marks, not an empty class grid";
  EXPECT_EQ(cit.getValue(), volume_grid::ToClassId(VoxelClass::kCliff));
  const openvdb::Vec3d world =
      class_grid->transform().indexToWorld(cit.getCoord());
  EXPECT_NEAR(world.x(), CliffVoxelPoint().x, test_utils::kVoxelSize);
  EXPECT_NEAR(world.y(), CliffVoxelPoint().y, test_utils::kVoxelSize);
  EXPECT_NEAR(world.z(), CliffVoxelPoint().z, test_utils::kVoxelSize);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_class_publish_and_clear");
  return RUN_ALL_TESTS();
}
