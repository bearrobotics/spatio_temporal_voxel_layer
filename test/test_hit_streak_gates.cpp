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

#include <cstddef>
#include <cstdint>
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

constexpr int kHitsToPromote = 3;
// Two consecutive misses break a streak.
constexpr int kMissesToBreak = 2;
constexpr std::size_t kCliffSourceIndex = 0;
constexpr std::size_t kOtherSourceIndex = 1;
constexpr std::size_t kCreationGatedSourceIndex = 2;
constexpr int kHitsToCreate = 3;
constexpr double kMarkAgeBeyondDecay = 2 * kDecayNever;
constexpr double kSecondsToBreak = 1.0;
constexpr double kFramePeriod = 0.1;
// A fixed stamp, so a slow test machine never breaks a streak by time.
const ros::Time kObservationStamp(100.0);

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// The gate counts observations per source, so tests number observations.
observation::MeasurementReading MakeReading(
    const std::vector<geometry_msgs::Point>& points, VoxelClass voxel_class,
    uint64_t observation_sequence_number,
    std::size_t source_index = kCliffSourceIndex,
    int misses_to_break = kMissesToBreak) {
  observation::MeasurementReading reading;
  reading._source_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(points));
  reading._cloud->header.stamp = kObservationStamp;
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  reading._voxel_class = voxel_class;
  reading._source_index = source_index;
  reading._observation_sequence_number = observation_sequence_number;
  reading._hit_streak_policy.misses_to_break = misses_to_break;
  reading._hit_streak_policy.seconds_to_break = kSecondsToBreak;
  return reading;
}

std::optional<VoxelClassTable> MakeCliffTable(
    double generic_decay_seconds, int cliff_hits_to_promote = kHitsToPromote,
    std::optional<double> cliff_store_at_z = std::nullopt) {
  VoxelClassTable::Config config;
  config.generic_decay_seconds = generic_decay_seconds;
  VoxelClassTable::Row row;
  row.voxel_class = VoxelClass::kCliff;
  row.policy.decay_seconds = kDecayNever;
  row.policy.priority = 10;
  row.policy.hits_to_promote = cliff_hits_to_promote;
  row.policy.cleared_by_frustums = true;
  row.policy.cleared_by_dynamic_obstacles = false;
  row.policy.cleared_by_footprint_clear = false;
  row.policy.cleared_by_front_blind_spot = false;
  row.policy.decays_in_inter_sensor_prism = false;
  row.policy.store_at_z = cliff_store_at_z;
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

void MarkCliffObservation(TestableGrid* grid,
                          uint64_t observation_sequence_number,
                          int misses_to_break = kMissesToBreak) {
  grid->Mark({MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                          observation_sequence_number, kCliffSourceIndex,
                          misses_to_break)});
}

// Observations 1 through observation_count with none missed.
void MarkCliffObservations(TestableGrid* grid, int observation_count) {
  for (int observation_sequence_number = 1;
       observation_sequence_number <= observation_count;
       ++observation_sequence_number) {
    MarkCliffObservation(grid, observation_sequence_number);
  }
}

void MarkCliffObservationAt(TestableGrid* grid,
                            uint64_t observation_sequence_number,
                            double seconds_after_first) {
  observation::MeasurementReading reading = MakeReading(
      {CliffVoxelPoint()}, VoxelClass::kCliff, observation_sequence_number);
  reading._cloud->header.stamp =
      kObservationStamp + ros::Duration(seconds_after_first);
  grid->Mark({reading});
}

observation::MeasurementReading MakeCreationGatedReading(
    const std::vector<geometry_msgs::Point>& points,
    uint64_t observation_sequence_number, int hits_to_create = kHitsToCreate,
    VoxelClass voxel_class = VoxelClass::kGeneric) {
  observation::MeasurementReading reading =
      MakeReading(points, voxel_class, observation_sequence_number,
                  kCreationGatedSourceIndex);
  reading._hit_streak_policy.hits_to_create = hits_to_create;
  return reading;
}

geometry_msgs::Point CreationGatedVoxelPoint() {
  return MakePoint(0.0, 0.0, 0.1);
}

void MarkCreationGatedObservation(TestableGrid* grid,
                                  uint64_t observation_sequence_number) {
  grid->Mark({MakeCreationGatedReading({CreationGatedVoxelPoint()},
                                       observation_sequence_number)});
}

// Observations 1 through observation_count with none missed.
void MarkCreationGatedObservations(TestableGrid* grid, int observation_count) {
  for (int observation_sequence_number = 1;
       observation_sequence_number <= observation_count;
       ++observation_sequence_number) {
    MarkCreationGatedObservation(grid, observation_sequence_number);
  }
}

void MarkCreationGatedObservationAt(TestableGrid* grid,
                                    uint64_t observation_sequence_number,
                                    double seconds_after_first) {
  observation::MeasurementReading reading = MakeCreationGatedReading(
      {CreationGatedVoxelPoint()}, observation_sequence_number);
  reading._cloud->header.stamp =
      kObservationStamp + ros::Duration(seconds_after_first);
  grid->Mark({reading});
}

void MarkCreationGatedCliffObservation(TestableGrid* grid,
                                       uint64_t observation_sequence_number,
                                       int hits_to_create) {
  grid->Mark({MakeCreationGatedReading({CliffVoxelPoint()},
                                       observation_sequence_number,
                                       hits_to_create, VoxelClass::kCliff)});
}

TEST(VoxelClassTableTest, HitsToPromoteBelowOneRejected) {
  EXPECT_FALSE(
      MakeCliffTable(kDecayNever, /*cliff_hits_to_promote=*/0).has_value());
}

TEST(VoxelClassTableTest, NonFiniteStoreAtZRejected) {
  EXPECT_FALSE(MakeCliffTable(kDecayNever, kHitsToPromote,
                              std::numeric_limits<double>::quiet_NaN())
                   .has_value());
}

TEST(PromotionGateTest, UnpromotedMarkStaysGeneric) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffObservations(grid.get(), kHitsToPromote - 1);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "an unpromoted cliff must still be an obstacle right away";
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric);
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(PromotionGateTest, ConsecutiveHitsPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffObservations(grid.get(), kHitsToPromote);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "a promoted voxel must stop holding a streak";
  EXPECT_TRUE(grid->IsClassGridConsistent());
}

TEST(PromotionGateTest, PointsInOneObservationCountAsOneHit) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  grid->Mark(
      {MakeReading({CliffVoxelPoint(), MakePoint(0.001, 0.0, 0.1),
                    MakePoint(0.0, 0.001, 0.1)},
                   VoxelClass::kCliff, /*observation_sequence_number=*/1)});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "one observation is one hit however many points land in the voxel";
}

TEST(PromotionGateTest, RepeatedObservationDoesNotPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const observation::MeasurementReading repeated_reading =
      MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                  /*observation_sequence_number=*/1);

  for (int delivery = 0; delivery < kHitsToPromote + 1; ++delivery) {
    grid->Mark({repeated_reading});
  }

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "re-reading one observation must not count as a new hit";
}

TEST(PromotionGateTest, MissesBelowMissesToBreakKeepTheStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  // Every other observation misses the voxel: one miss between hits.
  MarkCliffObservation(grid.get(), 1);
  MarkCliffObservation(grid.get(), 3);
  MarkCliffObservation(grid.get(), 5);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "misses below misses_to_break must keep the streak";
}

TEST(PromotionGateTest, MissesReachingMissesToBreakStartANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);

  // Two consecutive misses break the streak.
  MarkCliffObservation(grid.get(), kHitsToPromote + 2);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "hits from before the break must not complete a promotion";

  MarkCliffObservation(grid.get(), kHitsToPromote + 3);
  MarkCliffObservation(grid.get(), kHitsToPromote + 4);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "the observation after the break must start a new streak";
}

TEST(PromotionGateTest, MissesToBreakOfOneNeedsEveryObservation) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffObservation(grid.get(), 1, /*misses_to_break=*/1);
  MarkCliffObservation(grid.get(), 2, /*misses_to_break=*/1);
  MarkCliffObservation(grid.get(), 4, /*misses_to_break=*/1);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "with misses_to_break 1 a single miss must start a new streak";
}

TEST(PromotionGateTest, HitsFartherApartThanSecondsToBreakStartANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservationAt(grid.get(), 1, 0.0);
  MarkCliffObservationAt(grid.get(), 2, kFramePeriod);

  // A camera stall: the next hit comes more than seconds_to_break later.
  const double after_stall = kFramePeriod + kSecondsToBreak + kFramePeriod;
  MarkCliffObservationAt(grid.get(), 3, after_stall);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "hits from before the stall must not complete a promotion";

  MarkCliffObservationAt(grid.get(), 4, after_stall + kFramePeriod);
  MarkCliffObservationAt(grid.get(), 5, after_stall + 2 * kFramePeriod);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "the hit after the stall must start a new streak";
}

// A buffer with observation_persistence hands old observations over again.
TEST(PromotionGateTest, OlderObservationReadAgainIsNotCounted) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservation(grid.get(), 5);
  MarkCliffObservation(grid.get(), 6);

  MarkCliffObservation(grid.get(), 2);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "an older observation must not count as a new hit";

  MarkCliffObservation(grid.get(), 7);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "an older observation must not break the streak either";
}

// The buffer lists observations newest first, so a batch reads [3, 2, 1].
TEST(PromotionGateTest, PersistedBatchesNewestFirstPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const auto reading = [](uint64_t observation_sequence_number) {
    return MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                       observation_sequence_number);
  };

  grid->Mark({reading(1)});
  grid->Mark({reading(2), reading(1)});
  grid->Mark({reading(3), reading(2), reading(1)});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "re-read observations must neither count twice nor break the streak";
}

// With observation_persistence, one read can carry two unread observations.
TEST(PromotionGateTest, NewestFirstBatchOfUnreadObservationsPromotes) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const auto reading = [](uint64_t observation_sequence_number) {
    return MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                       observation_sequence_number, kCliffSourceIndex,
                       /*misses_to_break=*/1);
  };

  grid->Mark({reading(2), reading(1)});
  grid->Mark({reading(4), reading(3)});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "observations read together must count as consecutive, whatever "
         "their order";
}

TEST(PromotionGateTest, UnnumberedObservationStaysGenericAndStartsNoStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  for (int delivery = 0; delivery < kHitsToPromote; ++delivery) {
    MarkCliffObservation(grid.get(), /*observation_sequence_number=*/0);
  }

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "an observation the buffer never numbered cannot promote anything";
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "an unnumbered observation must not start a streak";
}

TEST(PromotionGateTest, UngatedClassPromotesOnFirstMark) {
  auto grid =
      MakeGrid(MakeCliffTable(kDecayNever, /*cliff_hits_to_promote=*/1));

  MarkCliffObservations(grid.get(), 1);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "an ungated class must not record hit streaks";
}

// The promoting observation writes the class grid partway through the points.
TEST(PromotionGateTest, PromotingObservationErasesTheStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);

  grid->Mark({MakeReading({CliffVoxelPoint(), MakePoint(0.001, 0.0, 0.1)},
                          VoxelClass::kCliff, kHitsToPromote)});

  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "a second point of the promoting observation must not start a "
         "new streak";
}

// Sequence numbers belong to a source, so sources cannot share one streak.
TEST(PromotionGateTest, SourcesCountTheirOwnObservations) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCliffObservations(grid.get(), kHitsToPromote - 1);
  grid->Mark(
      {MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                   /*observation_sequence_number=*/1, kOtherSourceIndex)});

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "another source's observation must not complete this source's streak";
  EXPECT_EQ(grid->GetHitStreakCount(), 2u)
      << "each source keeps its own streak on the voxel";
}

// Re-gating here would downgrade the mark and the cliff would expire.
TEST(PromotionGateTest, PromotedCliffRemarkSkipsTheGate) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), kHitsToPromote);
  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);

  MarkCliffObservation(grid.get(), kHitsToPromote + 1);

  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "re-observing a promoted cliff must not reopen the gate";
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);
}

TEST(PromotionGateTest, UnpromotedMarkExpiresOnGenericDecay) {
  auto grid = MakeGrid(MakeCliffTable(kDecayExpired));
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);
  ASSERT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));

  RunDecayPass(grid.get());

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a phantom that is never promoted must die on the generic timer";
}

TEST(PromotionGateTest, PromotedCliffSurvivesGenericDecayPass) {
  auto grid = MakeGrid(MakeCliffTable(kDecayExpired));
  MarkCliffObservations(grid.get(), kHitsToPromote);
  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff);

  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a promoted cliff must live on the cliff timer, not the generic one";
}

// Erasure judges each source by its own newest observation in the batch.
TEST(PromotionGateTest, OtherSourcesObservationsDoNotEraseTheStreak) {
  const observation::MeasurementReading far_ahead_reading =
      MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kCliff,
                  /*observation_sequence_number=*/100, kOtherSourceIndex);
  const observation::MeasurementReading first_reading =
      MakeReading({CliffVoxelPoint()}, VoxelClass::kCliff,
                  /*observation_sequence_number=*/1);

  using Batch = std::vector<observation::MeasurementReading>;
  for (const Batch& batch : {Batch{far_ahead_reading, first_reading},
                             Batch{first_reading, far_ahead_reading}}) {
    auto grid = MakeGrid(MakeCliffTable(kDecayNever));
    grid->Mark(batch);
    EXPECT_EQ(grid->GetHitStreakCount(), 2u)
        << "both sources keep the streak they just recorded";
  }
}

TEST(PromotionGateTest, StreaksTheNextHitCanContinueSurviveTheBatch) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), 1);
  ASSERT_EQ(grid->GetHitStreakCount(), 1u);

  // The first voxel missed one observation: the streak can still continue.
  grid->Mark({MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kCliff,
                          /*observation_sequence_number=*/kMissesToBreak)});

  EXPECT_EQ(grid->GetHitStreakCount(), 2u)
      << "a streak the next observation can continue must stay";
}

TEST(PromotionGateTest, BrokenStreaksAreErasedAfterTheBatch) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), 1);
  ASSERT_EQ(grid->GetHitStreakCount(), 1u);

  // The first voxel missed two observations: the next hit starts a new streak.
  grid->Mark({MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kCliff,
                          /*observation_sequence_number=*/1 + kMissesToBreak)});

  EXPECT_EQ(grid->GetHitStreakCount(), 1u) << "only the fresh streak survives";
}

TEST(PromotionGateTest, StreaksPastSecondsToBreakAreErasedAfterTheBatch) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservationAt(grid.get(), 1, 0.0);
  ASSERT_EQ(grid->GetHitStreakCount(), 1u);

  // The next observation misses the first voxel after a long stall.
  observation::MeasurementReading reading =
      MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kCliff,
                  /*observation_sequence_number=*/2);
  reading._cloud->header.stamp =
      kObservationStamp + ros::Duration(kSecondsToBreak + kFramePeriod);
  grid->Mark({reading});

  EXPECT_EQ(grid->GetHitStreakCount(), 1u) << "only the fresh streak survives";
}

// One hit short, so a promotion after the batch means the streak survived.
TEST(PromotionGateTest, GenericReadingDoesNotEraseTheStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), 1);

  // Far ahead; only a gated reading may erase streaks.
  grid->Mark(
      {MakeReading({MakePoint(1.0, 0.0, 0.1)}, VoxelClass::kGeneric,
                   /*observation_sequence_number=*/100, kOtherSourceIndex)});
  MarkCliffObservation(grid.get(), 2);
  MarkCliffObservation(grid.get(), 3);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "a generic reading's observation must not break a cliff streak";
}

TEST(PromotionGateTest, ResetGridErasesHitStreaks) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);

  ASSERT_TRUE(grid->ResetGrid());
  MarkCliffObservation(grid.get(), kHitsToPromote);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "hits from before a reset must not complete a promotion";
}

TEST(PromotionGateTest, ResetGridAreaErasesHitStreaks) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);

  grid->ResetGridArea(volume_grid::occupany_cell(-1.0, -1.0),
                      volume_grid::occupany_cell(1.0, 1.0),
                      /*invert_area=*/true);
  MarkCliffObservation(grid.get(), kHitsToPromote);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric)
      << "hits from before a reset must not complete a promotion";
}

TEST(CreationGateTest, SingleObservationNoiseCreatesNoVoxel) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCreationGatedObservation(grid.get(), 1);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a voxel seen in one observation must not become an obstacle";
}

TEST(CreationGateTest, VoxelAppearsOnTheHitsToCreateHit) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservations(grid.get(), kHitsToCreate - 1);
  ASSERT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));

  MarkCreationGatedObservation(grid.get(), kHitsToCreate);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_EQ(grid->GetClassAt(CreationGatedVoxelPoint()), VoxelClass::kGeneric);
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "a created voxel with no promotion ahead must not keep a streak";
}

TEST(CreationGateTest, PointsInOneObservationCountAsOneHit) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  grid->Mark({MakeCreationGatedReading(
      {CreationGatedVoxelPoint(), MakePoint(0.001, 0.0, 0.1),
       MakePoint(0.0, 0.001, 0.1)},
      /*observation_sequence_number=*/1)});

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "one observation counts once however many points hit the voxel";
}

TEST(CreationGateTest, MissesBelowMissesToBreakKeepTheStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  // One miss between hits stays below kMissesToBreak.
  MarkCreationGatedObservation(grid.get(), 1);
  MarkCreationGatedObservation(grid.get(), 3);
  MarkCreationGatedObservation(grid.get(), 5);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a miss below misses_to_break must keep the streak";
}

TEST(CreationGateTest, MissesReachingMissesToBreakStartANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservations(grid.get(), kHitsToCreate - 1);

  const int first_hit_after_break = kHitsToCreate - 1 + kMissesToBreak + 1;
  MarkCreationGatedObservation(grid.get(), first_hit_after_break);
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "hits from before the break must not create the voxel";

  MarkCreationGatedObservation(grid.get(), first_hit_after_break + 1);
  MarkCreationGatedObservation(grid.get(), first_hit_after_break + 2);
  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "the hit after the break must start a new streak";
}

TEST(CreationGateTest, HitsFartherApartThanSecondsToBreakStartANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservationAt(grid.get(), 1, 0.0);
  MarkCreationGatedObservationAt(grid.get(), 2, kFramePeriod);

  // A camera stall: the next hit comes more than seconds_to_break later.
  const double after_stall = kFramePeriod + kSecondsToBreak + kFramePeriod;
  MarkCreationGatedObservationAt(grid.get(), 3, after_stall);
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "hits from before the stall must not create the voxel";

  MarkCreationGatedObservationAt(grid.get(), 4, after_stall + kFramePeriod);
  MarkCreationGatedObservationAt(grid.get(), 5, after_stall + 2 * kFramePeriod);
  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "the hit after the stall must start a new streak";
}

TEST(CreationGateTest, StallShorterThanSecondsToBreakKeepsTheStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservationAt(grid.get(), 1, 0.0);
  MarkCreationGatedObservationAt(grid.get(), 2, kFramePeriod);

  MarkCreationGatedObservationAt(grid.get(), 3, kSecondsToBreak);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a stall shorter than seconds_to_break must keep the streak";
}

TEST(CreationGateTest, HitsExactlySecondsToBreakApartStartANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservationAt(grid.get(), 1, 0.0);
  MarkCreationGatedObservationAt(grid.get(), 2, 0.5);

  MarkCreationGatedObservationAt(grid.get(), 3, 0.5 + kSecondsToBreak);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a gap of seconds_to_break must break the streak, as "
         "misses_to_break misses do";
}

TEST(CreationGateTest, HitStampedBeforeTheLastHitStartsANewStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservationAt(grid.get(), 1, 0.0);
  MarkCreationGatedObservationAt(grid.get(), 2, kFramePeriod);

  // The clock jumped back, so the stamps no longer measure the gap.
  MarkCreationGatedObservationAt(grid.get(), 3, -kSecondsToBreak);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a hit stamped before the last hit must start a new streak";
}

TEST(CreationGateTest, MarkedVoxelSkipsTheGateAndRefreshes) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  grid->Mark(
      {MakeReading({CreationGatedVoxelPoint()}, VoxelClass::kGeneric,
                   /*observation_sequence_number=*/1, kOtherSourceIndex)});
  grid->AgeMark(CreationGatedVoxelPoint(), kMarkAgeBeyondDecay);

  MarkCreationGatedObservation(grid.get(), 1);
  RunDecayPass(grid.get());

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a hit on a marked voxel must refresh the voxel at once";
  EXPECT_EQ(grid->GetHitStreakCount(), 0u)
      << "a marked voxel needs no streak to stay marked";
}

TEST(CreationGateTest, VoxelRemovedByDecayMustPassTheGateAgain) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservations(grid.get(), kHitsToCreate);
  grid->AgeMark(CreationGatedVoxelPoint(), kMarkAgeBeyondDecay);
  RunDecayPass(grid.get());
  ASSERT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));

  MarkCreationGatedObservation(grid.get(), kHitsToCreate + 1);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "a voxel removed by decay must earn hits_to_create again";
}

TEST(CreationGateTest, UnnumberedObservationCreatesTheVoxelAtOnce) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));

  MarkCreationGatedObservation(grid.get(), /*observation_sequence_number=*/0);

  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "the creation gate must fail open on an unnumbered observation";
  EXPECT_EQ(grid->GetHitStreakCount(), 0u);
}

TEST(CreationGateTest, CliffSourceCreatesThenPromotesOnOneStreak) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const int hits_to_create = kHitsToPromote - 1;

  MarkCreationGatedCliffObservation(grid.get(), 1, hits_to_create);
  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));

  MarkCreationGatedCliffObservation(grid.get(), 2, hits_to_create);
  EXPECT_TRUE(HasVoxelNear(grid.get(), 0.0, 0.0));
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric);

  MarkCreationGatedCliffObservation(grid.get(), 3, hits_to_create);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "the hits that created the voxel must count toward the promotion";
  EXPECT_EQ(grid->GetHitStreakCount(), 0u);
}

TEST(CreationGateTest, CreatingHitAlsoPromotesWhenPromotionNeedsNoMoreHits) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const int hits_to_create = kHitsToPromote;

  MarkCreationGatedCliffObservation(grid.get(), 1, hits_to_create);
  MarkCreationGatedCliffObservation(grid.get(), 2, hits_to_create);
  ASSERT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));

  MarkCreationGatedCliffObservation(grid.get(), 3, hits_to_create);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "the voxel must go from unmarked straight to cliff";
}

TEST(CreationGateTest, DecayedVoxelWithAPromotionStreakReturnsOnTheNextHit) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  const int hits_to_create = kHitsToPromote - 1;
  MarkCreationGatedCliffObservation(grid.get(), 1, hits_to_create);
  MarkCreationGatedCliffObservation(grid.get(), 2, hits_to_create);
  ASSERT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kGeneric);
  grid->AgeMark(CliffVoxelPoint(), kMarkAgeBeyondDecay);
  RunDecayPass(grid.get());
  ASSERT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0));

  MarkCreationGatedCliffObservation(grid.get(), 3, hits_to_create);

  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "a streak kept for promotion must pass the creation gate at once";
}

TEST(CreationGateTest, BrokenCreationStreaksAreErasedAfterTheBatch) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservation(grid.get(), 1);
  ASSERT_EQ(grid->GetHitStreakCount(), 1u);

  // The first voxel missed two observations: the next hit starts a new streak.
  grid->Mark({MakeCreationGatedReading(
      {MakePoint(1.0, 0.0, 0.1)},
      /*observation_sequence_number=*/1 + kMissesToBreak)});

  EXPECT_EQ(grid->GetHitStreakCount(), 1u) << "only the fresh streak survives";
}

TEST(CreationGateTest, ErasedSourceStreaksStartCreationOver) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservations(grid.get(), kHitsToCreate - 1);

  grid->EraseHitStreaksOfSource(kCreationGatedSourceIndex);
  MarkCreationGatedObservation(grid.get(), kHitsToCreate);

  EXPECT_FALSE(HasVoxelNear(grid.get(), 0.0, 0.0))
      << "hits from before the streaks were erased must not count";
}

TEST(CreationGateTest, ErasingOneSourceKeepsTheOtherSourcesStreaks) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservation(grid.get(), 1);
  MarkCliffObservations(grid.get(), kHitsToPromote - 1);
  ASSERT_EQ(grid->GetHitStreakCount(), 2u);

  grid->EraseHitStreaksOfSource(kCreationGatedSourceIndex);
  ASSERT_EQ(grid->GetHitStreakCount(), 1u);
  MarkCliffObservation(grid.get(), kHitsToPromote);
  EXPECT_EQ(grid->GetClassAt(CliffVoxelPoint()), VoxelClass::kCliff)
      << "the cliff source's streak must survive";
}

TEST(CreationGateTest, ErasingAllHitStreaksDropsEverySourcesStreaks) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever));
  MarkCreationGatedObservation(grid.get(), 1);
  MarkCliffObservations(grid.get(), 1);
  ASSERT_EQ(grid->GetHitStreakCount(), 2u);

  grid->EraseAllHitStreaks();
  EXPECT_EQ(grid->GetHitStreakCount(), 0u);
}

TEST(ClassStoreAtZTest, CliffMarksAtAnyHeightLandInTheStoreAtZLayerAndPromote) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever, kHitsToPromote, 0.0));
  const std::vector<geometry_msgs::Point> points = {MakePoint(0.0, 0.0, -0.3),
                                                    MakePoint(0.2, 0.0, 0.01),
                                                    MakePoint(0.4, 0.0, -1e-7)};

  for (int observation_sequence_number = 1;
       observation_sequence_number <= kHitsToPromote;
       ++observation_sequence_number) {
    grid->Mark(
        {MakeReading(points, VoxelClass::kCliff, observation_sequence_number)});
  }

  for (const geometry_msgs::Point& point : points) {
    const std::vector<openvdb::Vec3d> voxels =
        grid->GetVoxelsAtXY(point.x, point.y, kQueryTolerance);
    ASSERT_EQ(voxels.size(), 1u) << "x=" << point.x;
    EXPECT_NEAR(voxels[0].z(), test_utils::kVoxelSize / 2.0, 1e-4)
        << "x=" << point.x;
    EXPECT_EQ(grid->GetClassAt(MakePoint(point.x, point.y, 0.0)),
              VoxelClass::kCliff)
        << "x=" << point.x;
  }
}

TEST(ClassStoreAtZTest, GenericSourceKeepsItsOwnHeight) {
  auto grid = MakeGrid(MakeCliffTable(kDecayNever, kHitsToPromote, 0.0));

  grid->Mark({MakeReading({MakePoint(0.0, 0.0, -0.3)}, VoxelClass::kGeneric,
                          /*observation_sequence_number=*/1)});

  const std::vector<openvdb::Vec3d> voxels =
      grid->GetVoxelsAtXY(0.0, 0.0, kQueryTolerance);
  ASSERT_EQ(voxels.size(), 1u);
  EXPECT_LT(voxels[0].z(), 0.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hit_streak_gates");
  return RUN_ALL_TESTS();
}
