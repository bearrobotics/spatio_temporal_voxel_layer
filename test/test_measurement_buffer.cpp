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
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/measurement_buffer.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/passthrough_filter.h"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using spatio_temporal_voxel_layer::PassthroughFilter;
using volume_grid::VoxelClass;

constexpr char kGlobalFrame[] = "map";
constexpr char kSourceName[] = "camera_source";
constexpr char kCliffSourceName[] = "gemini_down_cliff";
constexpr char kSensorFrame[] = "camera";
constexpr double kSensorX = 1.0;
constexpr double kSensorY = 2.0;
constexpr std::size_t kSourceIndex = 2;
constexpr int kHitsToCreate = 3;
constexpr int kMissesToBreak = 2;
const ros::Time kCloudStamp(1.0);

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

observation::HitStreakPolicy MakeHitStreakPolicy() {
  observation::HitStreakPolicy hit_streak_policy;
  hit_streak_policy.hits_to_create = kHitsToCreate;
  hit_streak_policy.misses_to_break = kMissesToBreak;
  return hit_streak_policy;
}

class MeasurementBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    geometry_msgs::TransformStamped sensor_in_map;
    sensor_in_map.header.frame_id = kGlobalFrame;
    sensor_in_map.header.stamp = kCloudStamp;
    sensor_in_map.child_frame_id = kSensorFrame;
    sensor_in_map.transform.translation.x = kSensorX;
    sensor_in_map.transform.translation.y = kSensorY;
    sensor_in_map.transform.rotation.w = 1.0;
    tf_buffer_.setTransform(sensor_in_map, "test", /*is_static=*/true);
  }

  std::unique_ptr<buffer::MeasurementBuffer> MakeBuffer(
      bool marking, bool clearing, VoxelClass voxel_class,
      const std::string& source_name = kSourceName,
      double observation_keep_time = 0.0) {
    // Zero keep time and rate: keep only the latest cloud, never check
    // staleness.
    return std::make_unique<buffer::MeasurementBuffer>(
        "/unused/points", source_name, observation_keep_time,
        /*expected_update_rate=*/0.0, /*obstacle_range=*/5.0, tf_buffer_,
        kGlobalFrame, kSensorFrame, /*tf_tolerance=*/0.1,
        /*decay_acceleration=*/0.0, marking, clearing, voxel_class,
        kSourceIndex, MakeHitStreakPolicy(), test_utils::kVoxelSize,
        std::make_unique<PassthroughFilter>(),
        /*enabled=*/true, /*clear_buffer_after_reading=*/false,
        FrustumFactoryFactory::FrustumFactory{});
  }

  static sensor_msgs::PointCloud2 MakeSensorCloud(
      const std::vector<geometry_msgs::Point>& points,
      const ros::Time& stamp = kCloudStamp) {
    sensor_msgs::PointCloud2 cloud = test_utils::MakePointCloud(points);
    cloud.header.frame_id = kSensorFrame;
    cloud.header.stamp = stamp;
    return cloud;
  }

  static std::vector<observation::MeasurementReading> MarkingReadingsOf(
      buffer::MeasurementBuffer& buffer) {
    std::vector<observation::MeasurementReading> readings;
    buffer.GetMarkingReadings(readings);
    return readings;
  }

  static std::vector<observation::MeasurementReading> ClearingReadingsOf(
      buffer::MeasurementBuffer& buffer) {
    std::vector<observation::MeasurementReading> readings;
    buffer.GetReadings(readings);
    return readings;
  }

  // ros::Time::now() needs a started node; the buffer stamps its updates.
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
};

TEST_F(MeasurementBufferTest, ReadingCarriesTheSourceVoxelClass) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kCliff);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));

  const std::vector<observation::MeasurementReading> readings =
      MarkingReadingsOf(*buffer);
  ASSERT_EQ(readings.size(), 1u);
  EXPECT_EQ(readings[0]._voxel_class, VoxelClass::kCliff);
  EXPECT_TRUE(readings[0]._marking);
  EXPECT_FALSE(readings[0]._clearing);
  EXPECT_DOUBLE_EQ(readings[0]._origin.x, kSensorX);
  EXPECT_DOUBLE_EQ(readings[0]._origin.y, kSensorY);
}

// Two sources can share a sensor frame, so the reading must carry the source.
TEST_F(MeasurementBufferTest, ReadingCarriesSourceNameNotSensorFrame) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kGeneric,
                 kCliffSourceName);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(1.0, 0.0, 0.0)}));

  const std::vector<observation::MeasurementReading> readings =
      MarkingReadingsOf(*buffer);
  ASSERT_EQ(readings.size(), 1u);
  EXPECT_EQ(readings[0]._source_name, kCliffSourceName);
}

TEST_F(MeasurementBufferTest, ReadingCarriesTheSourceIndexAndHitStreakPolicy) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kCliff);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));

  const std::vector<observation::MeasurementReading> readings =
      MarkingReadingsOf(*buffer);
  ASSERT_EQ(readings.size(), 1u);
  EXPECT_EQ(readings[0]._source_index, kSourceIndex);
  EXPECT_EQ(readings[0]._hit_streak_policy.hits_to_create, kHitsToCreate);
  EXPECT_EQ(readings[0]._hit_streak_policy.misses_to_break, kMissesToBreak);
}

TEST_F(MeasurementBufferTest,
       ObservationsAreNumberedFromOneWhenReadForMarking) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kCliff);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));
  ASSERT_EQ(MarkingReadingsOf(*buffer).size(), 1u);
  EXPECT_EQ(MarkingReadingsOf(*buffer)[0]._observation_sequence_number, 1u);

  // A zero keep time keeps only the newest cloud.
  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));
  ASSERT_EQ(MarkingReadingsOf(*buffer).size(), 1u);
  EXPECT_EQ(MarkingReadingsOf(*buffer)[0]._observation_sequence_number, 2u);
}

// An observation replaced before a read was never evaluated, so not a miss.
TEST_F(MeasurementBufferTest,
       AnObservationReplacedBeforeAMarkingReadIsNotNumbered) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kCliff);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));
  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));

  const std::vector<observation::MeasurementReading> readings =
      MarkingReadingsOf(*buffer);
  ASSERT_EQ(readings.size(), 1u);
  EXPECT_EQ(readings[0]._observation_sequence_number, 1u);
}

// The clearing pass reads a marking source too; only marking may number.
TEST_F(MeasurementBufferTest, AClearingReadLeavesObservationsUnnumbered) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/true, VoxelClass::kCliff);

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}));

  const std::vector<observation::MeasurementReading> clearing_readings =
      ClearingReadingsOf(*buffer);
  ASSERT_EQ(clearing_readings.size(), 1u);
  EXPECT_EQ(clearing_readings[0]._observation_sequence_number, 0u)
      << "a clearing read must not spend a sequence number";
  EXPECT_EQ(MarkingReadingsOf(*buffer)[0]._observation_sequence_number, 1u);
}

TEST_F(MeasurementBufferTest, AReadObservationKeepsItsNumberWhenReadAgain) {
  std::unique_ptr<buffer::MeasurementBuffer> buffer =
      MakeBuffer(/*marking=*/true, /*clearing=*/false, VoxelClass::kCliff,
                 kSourceName, /*observation_keep_time=*/10.0);
  // The keep time is measured from the wall clock, so stamps must be fresh.
  const ros::Time now = ros::Time::now();

  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.5, 0.0, 0.1)}, now));
  ASSERT_EQ(MarkingReadingsOf(*buffer).size(), 1u);
  buffer->BufferROSCloud(MakeSensorCloud({MakePoint(0.6, 0.0, 0.1)}, now));

  const std::vector<observation::MeasurementReading> readings =
      MarkingReadingsOf(*buffer);
  ASSERT_EQ(readings.size(), 2u)
      << "both observations are inside the keep time";
  EXPECT_EQ(readings[0]._observation_sequence_number, 2u)
      << "the newest observation is listed first";
  EXPECT_EQ(readings[1]._observation_sequence_number, 1u)
      << "a second read must not renumber the older observation";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_measurement_buffer");
  return RUN_ALL_TESTS();
}
