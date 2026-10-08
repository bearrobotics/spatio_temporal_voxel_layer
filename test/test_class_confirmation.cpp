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

#include <limits>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/Point.h"
#include "gtest/gtest.h"
#include "openvdb/openvdb.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using test_utils::TestableGrid;
using volume_grid::SpatioTemporalVoxelGrid;
using volume_grid::VoxelClass;
using volume_grid::VoxelClassTable;

constexpr double kQueryTolerance = 0.07;
constexpr double kDecayExpired = -1.0;
constexpr double kDecayNever = 1000.0;

constexpr int kFramesToConfirm = 3;
constexpr double kWindowSeconds = 1.0;
// Inside kWindowSeconds, so frames accumulate.
constexpr double kFrameGapSeconds = 0.1;

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// The gate counts stamps, so tests set stamps instead of using the clock.
observation::MeasurementReading MakeReading(
    const std::vector<geometry_msgs::Point>& points, VoxelClass voxel_class,
    const ros::Time& stamp) {
  observation::MeasurementReading reading;
  reading._sensor_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(points));
  reading._cloud->header.stamp = stamp;
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  reading._voxel_class = voxel_class;
  return reading;
}

std::optional<VoxelClassTable> MakeCliffTable(
    double generic_decay_seconds,
    int cliff_frames_to_confirm = kFramesToConfirm,
    double cliff_window_seconds = kWindowSeconds) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = generic_decay_seconds;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = kDecayNever;
  row.policy.priority = 10;
  row.policy.confirmation_frames = cliff_frames_to_confirm;
  row.policy.confirmation_window_seconds = cliff_window_seconds;
  row.policy.cleared_by_frustums = true;
  row.policy.cleared_by_dynamic_obstacles = false;
  row.policy.cleared_by_footprint_clear = false;
  row.policy.cleared_by_front_blind_spot = false;
  row.policy.decays_in_inter_sensor_prism = false;
  config.rows.push_back(row);
  return VoxelClassTable::Create(config);
}

std::unique_ptr<TestableGrid> MakeGrid(
    std::optional<VoxelClassTable> class_table) {
  openvdb::initialize();
  // Unused: voxel_decay only seeds a table the caller did not supply.
  return std::make_unique<TestableGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, volume_grid::LINEAR,
      /*voxel_decay=*/0.0, /*pub_voxels=*/false,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(), /*front_blind_spot=*/nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker(), std::move(class_table));
}

void RunDecayPass(SpatioTemporalVoxelGrid* grid) {
  std::unordered_set<volume_grid::occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  grid->ClearFrustums({}, cleared_cells, dynamic_obstacle_readings,
                      robot_motion_readings);
}

bool HasVoxelNear(SpatioTemporalVoxelGrid* grid, double x, double y) {
  return !grid->GetVoxelsAtXY(x, y, kQueryTolerance).empty();
}

geometry_msgs::Point CliffVoxelPoint() { return MakePoint(0.0, 0.0, 0.1); }

ros::Time StampAt(int frame_index) {
  return ros::Time(1000.0) + ros::Duration(frame_index * kFrameGapSeconds);
}

void MarkCliffFrames(TestableGrid* grid, int frame_count) {
  for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
    grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                            StampAt(frame_index))});
  }
}

TEST(ClassConfirmationTableTest, FramesBelowOneRejected) {
  EXPECT_FALSE(
      MakeCliffTable(kDecayNever, /*cliff_frames_to_confirm=*/0, kWindowSeconds)
          .has_value());
}

TEST(ClassConfirmationTableTest, GatedClassWithoutWindowRejected) {
  EXPECT_FALSE(MakeCliffTable(kDecayNever, kFramesToConfirm,
                              /*cliff_window_seconds=*/0.0)
                   .has_value());
}

TEST(ClassConfirmationTableTest, GatedClassWithInfiniteWindowRejected) {
  EXPECT_FALSE(MakeCliffTable(kDecayNever, kFramesToConfirm,
                              std::numeric_limits<double>::infinity())
                   .has_value());
}

TEST(ClassConfirmationTableTest, UngatedClassIgnoresWindow) {
  EXPECT_TRUE(MakeCliffTable(kDecayNever, /*cliff_frames_to_confirm=*/1,
                             /*cliff_window_seconds=*/0.0)
                  .has_value());
}

TEST(ClassConfirmationGridTest, UnconfirmedMarkStaysGeneric) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "an unconfirmed cliff must still be an obstacle right away";
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric);
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(ClassConfirmationGridTest, FramesWithinWindowPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffFrames(grid.get(), kFramesToConfirm);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetPendingPromotionCount(), 0u)
      << "a promoted voxel must stop holding evidence";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(ClassConfirmationGridTest, PointsInOneCloudCountAsOneFrame) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  grid->Mark({MakeReading({CliffVoxelPoint(), MakePoint(0.001, 0.0, 0.1),
                           MakePoint(0.0, 0.001, 0.1)},
                          VoxelClass::kCliff, StampAt(0))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "one cloud is one frame however many of its points hit the voxel";
}

TEST(ClassConfirmationGridTest, RepeatedStampDoesNotPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const observation::MeasurementReading repeated_reading =
      MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(0));

  for (int delivery = 0; delivery < kFramesToConfirm + 1; ++delivery) {
    grid->Mark({repeated_reading});
  }

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "re-reading one cloud must not count as fresh evidence";
}

TEST(ClassConfirmationGridTest, GapBeyondWindowResetsCount) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  const ros::Time stamp_after_gap =
      StampAt(kFramesToConfirm - 2) + ros::Duration(2.0 * kWindowSeconds);
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, stamp_after_gap)});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "evidence older than the window must not complete a promotion";
}

// A bag loop jumps far back, unlike ReorderedFramesWithinWindowPromote.
TEST(ClassConfirmationGridTest, BackwardJumpBeyondWindowResetsCount) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                          StampAt(0) - ros::Duration(2.0 * kWindowSeconds))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "evidence from before the jump must not complete a promotion";
}

TEST(ClassConfirmationGridTest, UngatedClassPromotesOnFirstMark) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever,
                                      /*cliff_frames_to_confirm=*/1,
                                      /*cliff_window_seconds=*/0.0));

  MarkCliffFrames(grid.get(), 1);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetPendingPromotionCount(), 0u)
      << "an ungated class must not allocate candidates";
}

// The promoting cloud writes the class grid partway through the points.
TEST(ClassConfirmationGridTest, PromotingCloudLeavesNoEvidenceBehind) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  grid->Mark({MakeReading({CliffVoxelPoint(), MakePoint(0.001, 0.0, 0.1)},
                          VoxelClass::kCliff, StampAt(kFramesToConfirm - 1))});

  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetPendingPromotionCount(), 0u)
      << "a second point of the promoting cloud must not restart the count";
}

// Sources feeding one class can be skewed, so frames arrive out of order.
TEST(ClassConfirmationGridTest, ReorderedFramesWithinWindowPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(2))});
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(0))});
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(1))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "frames inside the window are evidence whatever order they arrive in";
}

// Re-gating here would downgrade the mark and the cliff would expire.
TEST(ClassConfirmationGridTest, ConfirmedCliffRemarkSkipsTheGate) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm);
  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);

  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                          StampAt(kFramesToConfirm))});

  EXPECT_EQ(grid->GetPendingPromotionCount(), 0u)
      << "re-observing a confirmed cliff must not reopen the gate";
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
}

TEST(ClassConfirmationGridTest, UnconfirmedMarkExpiresOnGenericDecay) {
  auto grid = MakeGrid(MakeCliffTable(kDecayExpired));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));

  RunDecayPass(grid.get());

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a phantom that never confirms must die on the generic timer";
}

TEST(ClassConfirmationGridTest, PromotedCliffSurvivesGenericDecayPass) {
  auto grid = MakeGrid(MakeCliffTable(kDecayExpired));
  MarkCliffFrames(grid.get(), kFramesToConfirm);
  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);

  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a promoted cliff must live on the cliff timer, not the generic one";
}

// Expiry uses the last cloud stamp of a batch, whichever source it came from.
TEST(ClassConfirmationGridTest, LaggingSourceKeepsEvidenceInAMixedBatch) {
  const observation::MeasurementReading leading_reading =
      MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kCliff,
                  StampAt(0) + ros::Duration(2.0 * kWindowSeconds));
  const observation::MeasurementReading lagging_reading =
      MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(0));

  using Batch = std::vector<observation::MeasurementReading>;
  for (const Batch& batch : {Batch{leading_reading, lagging_reading},
                             Batch{lagging_reading, leading_reading}}) {
    auto grid = MakeGrid(MakeCliffTable(kDecayNever));
    grid->Mark(batch);
    EXPECT_EQ(grid->GetPendingPromotionCount(), 2u)
        << "both sources keep the frame they just recorded";
  }
}

TEST(ClassConfirmationGridTest, StaleCandidatesExpireAfterTheBatch) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), 1);
  ASSERT_EQ(grid->GetPendingPromotionCount(), 1u);

  // Past the expiry margin.
  const geometry_msgs::Point other_voxel_point = MakePoint(1.0, 0.0, 0.1);
  grid->Mark(
      {MakeReading({other_voxel_point}, VoxelClass::kCliff,
                   StampAt(0) + ros::Duration(2.0 * kWindowSeconds + 2.0))});

  EXPECT_EQ(grid->GetPendingPromotionCount(), 1u)
      << "only the fresh candidate survives";
}

// One frame short, so a promotion after the reset means evidence survived.
TEST(ClassConfirmationGridTest, GenericReadingDoesNotExpireEvidence) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), 1);

  // Far past the expiry margin; only a gated reading may age evidence.
  grid->Mark({MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kGeneric,
                          StampAt(0) + ros::Duration(100.0))});
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(1))});
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff, StampAt(2))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "a generic reading's stamp must not age cliff evidence";
}

TEST(ClassConfirmationGridTest, ResetGridDropsPendingEvidence) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  ASSERT_TRUE(grid->ResetGrid());
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                          StampAt(kFramesToConfirm))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "evidence from before a reset must not complete a promotion";
}

TEST(ClassConfirmationGridTest, ResetGridAreaDropsPendingEvidence) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffFrames(grid.get(), kFramesToConfirm - 1);

  grid->ResetGridArea(volume_grid::occupany_cell(-1.0, -1.0),
                      volume_grid::occupany_cell(1.0, 1.0),
                      /*invert_area=*/true);
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                          StampAt(kFramesToConfirm))});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "evidence from before a reset must not complete a promotion";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_class_confirmation");
  return RUN_ALL_TESTS();
}
