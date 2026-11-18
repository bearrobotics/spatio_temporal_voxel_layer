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

#include <bear_msgs/ClassMap.h>
#include <bear_msgs/ClearClassVoxels.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"
#include "test/test_utils.h"

namespace {

using spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer;
using spatio_temporal_voxel_layer::test_utils::MakePointCloud;
using spatio_temporal_voxel_layer::test_utils::MaskAt;

// The ns of the rosparam block in test_layer_voxel_class_params.test.
constexpr char kParamNamespace[] = "/stvl_layer_params_test";
constexpr char kGlobalFrame[] = "map";
constexpr unsigned int kMapCells = 20;
constexpr double kResolution = 0.05;
constexpr double kMapOrigin = -0.5;
constexpr char kClassifiedLayer[] = "classified";
constexpr char kGenericOnlyLayer[] = "generic_only";
constexpr char kCliffWithoutStoreAtZLayer[] = "cliff_without_store_at_z";
constexpr char kNanStoreAtZLayer[] = "nan_store_at_z";
constexpr char kStringStoreAtZLayer[] = "string_store_at_z";
constexpr char kStaleConfirmationBlockLayer[] = "stale_confirmation_block";
constexpr char kGatedSourceWithoutHitStreakLayer[] =
    "gated_source_without_hit_streak";
constexpr char kZeroMissesToBreakLayer[] = "zero_misses_to_break";
constexpr char kClearingOnlyGatedSourceLayer[] = "clearing_only_gated_source";
constexpr char kStringMissesToBreakLayer[] = "string_misses_to_break";
constexpr char kHitStreakWithoutMissesToBreakLayer[] =
    "hit_streak_without_misses_to_break";
constexpr char kHitStreakOnUngatedSourceLayer[] =
    "hit_streak_on_ungated_source";
constexpr char kCreationGatedSourceLayer[] = "creation_gated_source";
constexpr char kZeroHitsToCreateLayer[] = "zero_hits_to_create";
constexpr char kStringHitsToCreateLayer[] = "string_hits_to_create";
constexpr char kZeroSecondsToBreakLayer[] = "zero_seconds_to_break";
constexpr char kStringSecondsToBreakLayer[] = "string_seconds_to_break";
constexpr char kNanSecondsToBreakLayer[] = "nan_seconds_to_break";
constexpr char kUnusedHitStreakWarning[] = "hit_streak is unused";
// The layer advertises under <param namespace>/<layer name>.
constexpr char kCliffVoxelTopic[] =
    "/stvl_layer_params_test/classified/cliff_voxel_grid";
constexpr char kVehicleVoxelTopic[] =
    "/stvl_layer_params_test/classified/vehicle_voxel_grid";
constexpr char kHitStreakResetLayer[] = "hit_streak_reset";
constexpr char kGatedSourceTopic[] =
    "/stvl_layer_params_test/hit_streak_reset_frames";
// The layer serves toggle_enabled under <namespace>/<layer name>/<source>.
constexpr char kGatedSourceToggleService[] =
    "/stvl_layer_params_test/hit_streak_reset/gated_camera/toggle_enabled";
constexpr char kClassMapPublisherLayer[] = "class_map_publisher";
constexpr char kClassMapTopic[] =
    "/stvl_layer_params_test/class_map_publisher/class_map";
constexpr char kClassMapFramesTopic[] =
    "/stvl_layer_params_test/class_map_frames";
constexpr char kClassifiedClassMapTopic[] =
    "/stvl_layer_params_test/classified/class_map";
constexpr char kSmallClassMapWindowLayer[] = "small_class_map_window";
constexpr char kSmallClassMapWindowTopic[] =
    "/stvl_layer_params_test/small_class_map_window/class_map";
// Must match class_map_window_width_m of small_class_map_window.
constexpr double kSmallClassMapWindowWidthM = 4.0;
constexpr char kZeroClassMapWindowLayer[] = "zero_class_map_window";
constexpr char kNanClassMapWindowLayer[] = "nan_class_map_window";
constexpr char kOversizedClassMapWindowLayer[] = "oversized_class_map_window";
constexpr char kStringPublishClassMapLayer[] = "string_publish_class_map";
constexpr char kStringClassMapWindowLayer[] = "string_class_map_window";
constexpr char kClassMapType[] = "bear_msgs/ClassMap";
constexpr double kDefaultClassMapWindowWidthM = 20.0;
constexpr uint16_t kVehicleBit =
    volume_grid::ToClassBit(volume_grid::VoxelClass::kVehicle);

bool IsAdvertised(const std::string& topic_name, const std::string& datatype) {
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  for (const ros::master::TopicInfo& topic : topics) {
    if (topic.name == topic_name) {
      return topic.datatype == datatype;
    }
  }
  return false;
}

unsigned int CountCellsWithCost(const costmap_2d::Costmap2D& costmap,
                                unsigned char value) {
  unsigned int count = 0;
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
      if (costmap.getCost(i, j) == value) {
        ++count;
      }
    }
  }
  return count;
}

// Fixed stamps, so a slow test machine never breaks a streak by time.
const ros::Time kFirstFrameStamp(100.0);
constexpr double kFramePeriod = 0.1;

// A point inside the map, so every frame hits the same voxel.
constexpr double kHitX = 0.2;
constexpr double kHitY = 0.2;
constexpr double kHitZ = 0.3;

boost::shared_ptr<sensor_msgs::PointCloud2> MakeHitFrame(
    const ros::Time& stamp) {
  geometry_msgs::Point hit;
  hit.x = kHitX;
  hit.y = kHitY;
  hit.z = kHitZ;
  auto frame =
      boost::make_shared<sensor_msgs::PointCloud2>(MakePointCloud({hit}));
  frame->header.stamp = stamp;
  return frame;
}

class LayerVoxelClassParamsTest : public ::testing::Test {
 protected:
  LayerVoxelClassParamsTest()
      : parent_nh_(kParamNamespace),
        layered_costmap_(parent_nh_, kGlobalFrame, /*rolling_window=*/false,
                         /*track_unknown=*/false) {
    layered_costmap_.resizeMap(kMapCells, kMapCells, kResolution, kMapOrigin,
                               kMapOrigin);
  }

  // layer_name selects a config block in test_layer_voxel_class_params.yaml.
  boost::shared_ptr<SpatioTemporalVoxelLayer> InitializeLayer(
      const std::string& layer_name) {
    auto layer = boost::make_shared<SpatioTemporalVoxelLayer>();
    layered_costmap_.addPlugin(layer);
    layer->initialize(&layered_costmap_, layer_name, parent_nh_,
                      /*tf=*/nullptr);
    return layer;
  }

  // A missing required param shuts the node down instead of throwing.
  void ExpectInitializedAndEmpty(
      const boost::shared_ptr<SpatioTemporalVoxelLayer>& layer) {
    ASSERT_TRUE(ros::ok()) << "a required layer param is missing";

    layered_costmap_.updateMap(0.0, 0.0, 0.0);

    EXPECT_TRUE(layer->isCurrent());
    EXPECT_EQ(CountCellsWithCost(*layered_costmap_.getCostmap(),
                                 costmap_2d::LETHAL_OBSTACLE),
              0u);
  }

  bear_msgs::ClearClassVoxels::Response ClearClassVoxels(
      SpatioTemporalVoxelLayer& layer, const std::string& class_name) {
    bear_msgs::ClearClassVoxels::Request req;
    bear_msgs::ClearClassVoxels::Response resp;
    req.class_name = class_name;
    EXPECT_TRUE(layer.ClearClassVoxelsCallback(req, resp));
    return resp;
  }

  bool LastUpdateRedrewTheWholeMap() {
    unsigned int x0 = 0;
    unsigned int xn = 0;
    unsigned int y0 = 0;
    unsigned int yn = 0;
    layered_costmap_.getBounds(&x0, &xn, &y0, &yn);
    return x0 == 0 && y0 == 0 && xn == kMapCells && yn == kMapCells;
  }

  boost::shared_ptr<SpatioTemporalVoxelLayer> InitializeHitStreakResetLayer() {
    frame_publisher_ =
        parent_nh_.advertise<sensor_msgs::PointCloud2>(kGatedSourceTopic, 1);
    return InitializeLayer(kHitStreakResetLayer);
  }

  void PublishHitAndUpdate(double seconds_after_previous_frame = kFramePeriod) {
    next_frame_stamp_ += ros::Duration(seconds_after_previous_frame);
    // A shared pointer reaches an in-process subscriber before publish returns.
    frame_publisher_.publish(MakeHitFrame(next_frame_stamp_));
    // The tf filter queues a second callback, which buffers the frame.
    DrainCallbackQueue();
    layered_costmap_.updateMap(0.0, 0.0, 0.0);
  }

  // One spin runs only the subscriber, so the frame waits in the tf filter.
  void PublishHitIntoTfFilter() {
    next_frame_stamp_ += ros::Duration(kFramePeriod);
    frame_publisher_.publish(MakeHitFrame(next_frame_stamp_));
    ros::spinOnce();
  }

  void DrainCallbackQueue() {
    while (!ros::getGlobalCallbackQueue()->isEmpty()) {
      ros::spinOnce();
    }
  }

  // A spinner serves toggle_enabled while this thread waits for the reply.
  bool SetGatedSourceEnabled(bool enabled) {
    std_srvs::SetBool toggle;
    toggle.request.data = enabled;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    const bool is_called =
        ros::service::call(kGatedSourceToggleService, toggle);
    spinner.stop();
    return is_called && toggle.response.success;
  }

  // The costmap is built before marking, so a voxel shows one update later.
  unsigned int CountLethalCellsAfterNextUpdate() {
    layered_costmap_.updateMap(0.0, 0.0, 0.0);
    return CountCellsWithCost(*layered_costmap_.getCostmap(),
                              costmap_2d::LETHAL_OBSTACLE);
  }

  boost::shared_ptr<SpatioTemporalVoxelLayer> InitializeClassMapLayer(
      const std::string& layer_name, const std::string& class_map_topic) {
    class_map_subscriber_ = parent_nh_.subscribe(
        class_map_topic, 1, &LayerVoxelClassParamsTest::OnClassMap, this);
    frame_publisher_ =
        parent_nh_.advertise<sensor_msgs::PointCloud2>(kClassMapFramesTopic, 1);
    return InitializeLayer(layer_name);
  }

  void OnClassMap(const bear_msgs::ClassMap::ConstPtr& map) {
    last_class_map_ = map;
  }

  // Maps arrive via the global callback queue; wait for this updateMap's map.
  bear_msgs::ClassMap::ConstPtr UpdateAndTakeClassMap() {
    const ros::Time update_start = ros::Time::now();
    layered_costmap_.updateMap(0.0, 0.0, 0.0);
    while (!last_class_map_ || last_class_map_->header.stamp < update_start) {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
    }
    return last_class_map_;
  }

  ros::NodeHandle parent_nh_;
  costmap_2d::LayeredCostmap layered_costmap_;
  ros::Publisher frame_publisher_;
  ros::Subscriber class_map_subscriber_;
  bear_msgs::ClassMap::ConstPtr last_class_map_;
  ros::Time next_frame_stamp_ = kFirstFrameStamp;
};

TEST_F(LayerVoxelClassParamsTest, ClassifiedSourcesInitializeAndStartEmpty) {
  ExpectInitializedAndEmpty(InitializeLayer(kClassifiedLayer));
}

TEST_F(LayerVoxelClassParamsTest, SourcesDefaultToGenericWithoutVoxelClasses) {
  ExpectInitializedAndEmpty(InitializeLayer(kGenericOnlyLayer));
}

TEST_F(LayerVoxelClassParamsTest, ClassMapIsAdvertisedWhenAClassPublishesIt) {
  ExpectInitializedAndEmpty(InitializeLayer(kClassMapPublisherLayer));

  EXPECT_TRUE(IsAdvertised(kClassMapTopic, kClassMapType));
}

TEST_F(LayerVoxelClassParamsTest,
       ClassMapIsNotAdvertisedWithoutAPublishingClass) {
  ExpectInitializedAndEmpty(InitializeLayer(kClassifiedLayer));

  EXPECT_FALSE(IsAdvertised(kClassifiedClassMapTopic, kClassMapType));
}

TEST_F(LayerVoxelClassParamsTest, ClassMapReportsTheMarkedColumnToASubscriber) {
  InitializeClassMapLayer(kClassMapPublisherLayer, kClassMapTopic);
  ASSERT_TRUE(ros::ok());

  PublishHitAndUpdate();
  const bear_msgs::ClassMap::ConstPtr map = UpdateAndTakeClassMap();

  ASSERT_TRUE(map);
  EXPECT_EQ(map->header.frame_id, kGlobalFrame);
  EXPECT_FLOAT_EQ(map->info.resolution, kResolution);
  ASSERT_EQ(map->class_names.size(), volume_grid::kVoxelClassCount);
  EXPECT_EQ(map->class_names[static_cast<std::size_t>(
                volume_grid::VoxelClass::kVehicle)],
            "vehicle");
  EXPECT_EQ(map->class_names[static_cast<std::size_t>(
                volume_grid::VoxelClass::kCliff)],
            "");
  // The hit sits on a voxel edge; probe the voxel centre.
  EXPECT_EQ(MaskAt(*map, kHitX + kResolution / 2.0, kHitY + kResolution / 2.0),
            kVehicleBit);
  EXPECT_NEAR(map->window_min.x, -kDefaultClassMapWindowWidthM / 2.0,
              kResolution);
  EXPECT_NEAR(map->window_min.y, -kDefaultClassMapWindowWidthM / 2.0,
              kResolution);
  EXPECT_NEAR(map->window_max.x, kDefaultClassMapWindowWidthM / 2.0,
              kResolution);
  EXPECT_NEAR(map->window_max.y, kDefaultClassMapWindowWidthM / 2.0,
              kResolution);
}

TEST_F(LayerVoxelClassParamsTest, ClassMapWindowFollowsTheParam) {
  InitializeClassMapLayer(kSmallClassMapWindowLayer, kSmallClassMapWindowTopic);
  ASSERT_TRUE(ros::ok());

  const bear_msgs::ClassMap::ConstPtr map = UpdateAndTakeClassMap();

  ASSERT_TRUE(map);
  EXPECT_NEAR(map->window_min.x, -kSmallClassMapWindowWidthM / 2.0,
              kResolution);
  EXPECT_NEAR(map->window_min.y, -kSmallClassMapWindowWidthM / 2.0,
              kResolution);
  EXPECT_NEAR(map->window_max.x, kSmallClassMapWindowWidthM / 2.0, kResolution);
  EXPECT_NEAR(map->window_max.y, kSmallClassMapWindowWidthM / 2.0, kResolution);
}

TEST_F(LayerVoxelClassParamsTest, ZeroClassMapWindowTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kZeroClassMapWindowLayer),
               "class_map_window_width_m is 0; it must be at least 2 voxels");
}

TEST_F(LayerVoxelClassParamsTest, NonFiniteClassMapWindowTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kNanClassMapWindowLayer),
               "class_map_window_width_m is nan; it must be at least 2 voxels");
}

TEST_F(LayerVoxelClassParamsTest, OversizedClassMapWindowTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kOversizedClassMapWindowLayer),
               "class_map_window_width_m is 100.5; .* at most 100 m");
}

TEST_F(LayerVoxelClassParamsTest, MistypedPublishClassMapTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringPublishClassMapLayer),
               "publish_class_map must be a bool");
}

TEST_F(LayerVoxelClassParamsTest, MistypedClassMapWindowTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringClassMapWindowLayer),
               "class_map_window_width_m must be a number");
}

TEST_F(LayerVoxelClassParamsTest, CliffClassWithoutStoreAtZInitializes) {
  ExpectInitializedAndEmpty(InitializeLayer(kCliffWithoutStoreAtZLayer));
}

// The threadsafe style re-runs the test binary instead of forking the node.
TEST_F(LayerVoxelClassParamsTest, NonFiniteStoreAtZTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kNanStoreAtZLayer),
               "a finite store_at_z when set");
}

TEST_F(LayerVoxelClassParamsTest, MistypedStoreAtZTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringStoreAtZLayer),
               "store_at_z must be a number, got .*zero");
}

TEST_F(LayerVoxelClassParamsTest,
       LeftoverConfirmationBlockTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStaleConfirmationBlockLayer),
               "confirmation is no longer read");
}

TEST_F(LayerVoxelClassParamsTest,
       GatedSourceWithoutHitStreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kGatedSourceWithoutHitStreakLayer),
               "misses_to_break is required");
}

TEST_F(LayerVoxelClassParamsTest, ZeroMissesToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kZeroMissesToBreakLayer),
               "misses_to_break must be >= 1");
}

TEST_F(LayerVoxelClassParamsTest, MistypedMissesToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringMissesToBreakLayer),
               "misses_to_break must be an int, got .*2");
}

TEST_F(LayerVoxelClassParamsTest,
       HitStreakWithoutMissesToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kHitStreakWithoutMissesToBreakLayer),
               "hit_streak has no misses_to_break");
}

// Only marking readings reach the gate, so a clearing source needs no value.
TEST_F(LayerVoxelClassParamsTest, ClearingOnlyGatedSourceInitializes) {
  ExpectInitializedAndEmpty(InitializeLayer(kClearingOnlyGatedSourceLayer));
}

TEST_F(LayerVoxelClassParamsTest, CreationGatedSourceInitializes) {
  ExpectInitializedAndEmpty(InitializeLayer(kCreationGatedSourceLayer));
}

TEST_F(LayerVoxelClassParamsTest, ReactivatedLayerStaysEmptyAndCurrent) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeLayer(kCreationGatedSourceLayer);
  layer->deactivate();

  layer->activate();

  ExpectInitializedAndEmpty(layer);
}

TEST_F(LayerVoxelClassParamsTest, ReenabledSourceNeedsAFullHitStreakToCreate) {
  InitializeHitStreakResetLayer();
  PublishHitAndUpdate();
  PublishHitAndUpdate();
  ASSERT_EQ(CountLethalCellsAfterNextUpdate(), 0u);

  ASSERT_TRUE(SetGatedSourceEnabled(false));
  // An update while the source is off reads what the source's buffer holds.
  layered_costmap_.updateMap(0.0, 0.0, 0.0);
  ASSERT_TRUE(SetGatedSourceEnabled(true));

  PublishHitAndUpdate();
  PublishHitAndUpdate();
  EXPECT_EQ(CountLethalCellsAfterNextUpdate(), 0u)
      << "a frame from before the toggle counted toward the new hit streak";
  PublishHitAndUpdate();
  EXPECT_GT(CountLethalCellsAfterNextUpdate(), 0u);
}

TEST_F(LayerVoxelClassParamsTest, ReactivatedLayerNeedsAFullHitStreakToCreate) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeHitStreakResetLayer();
  PublishHitAndUpdate();
  PublishHitAndUpdate();
  ASSERT_EQ(CountLethalCellsAfterNextUpdate(), 0u);
  PublishHitIntoTfFilter();

  layer->deactivate();
  layer->activate();
  DrainCallbackQueue();
  // The first update after activation reads what the source's buffer holds.
  layered_costmap_.updateMap(0.0, 0.0, 0.0);

  PublishHitAndUpdate();
  PublishHitAndUpdate();
  EXPECT_EQ(CountLethalCellsAfterNextUpdate(), 0u)
      << "a frame from before the reactivation counted toward the new hit "
         "streak";
  PublishHitAndUpdate();
  EXPECT_GT(CountLethalCellsAfterNextUpdate(), 0u);
}

// The frame gap sits between the default seconds_to_break and the source's.
TEST_F(LayerVoxelClassParamsTest,
       ConfiguredSecondsToBreakKeepsSlowHitsInOneStreak) {
  InitializeHitStreakResetLayer();
  constexpr double kSecondsBetweenFrames = 1.5;

  PublishHitAndUpdate(kSecondsBetweenFrames);
  PublishHitAndUpdate(kSecondsBetweenFrames);
  PublishHitAndUpdate(kSecondsBetweenFrames);

  EXPECT_GT(CountLethalCellsAfterNextUpdate(), 0u)
      << "the default seconds_to_break would break the streak at every frame";
}

TEST_F(LayerVoxelClassParamsTest, ZeroHitsToCreateTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kZeroHitsToCreateLayer),
               "hits_to_create must be >= 1");
}

TEST_F(LayerVoxelClassParamsTest, MistypedHitsToCreateTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringHitsToCreateLayer),
               "hits_to_create must be an int, got .*3");
}

TEST_F(LayerVoxelClassParamsTest, ZeroSecondsToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kZeroSecondsToBreakLayer),
               "seconds_to_break must be a positive finite number, got .*0");
}

TEST_F(LayerVoxelClassParamsTest, MistypedSecondsToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kStringSecondsToBreakLayer),
               "seconds_to_break must be a number, got .*1\\.5");
}

TEST_F(LayerVoxelClassParamsTest, NonFiniteSecondsToBreakTerminatesAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_DEATH(InitializeLayer(kNanSecondsToBreakLayer),
               "seconds_to_break must be a positive finite number, got .*nan");
}

// The first test shuts the node down, which silences rosconsole for the rest
// of the process, so the warning is read from a child process.
TEST_F(LayerVoxelClassParamsTest, HitStreakOnAnUngatedSourceWarnsAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_EXIT(
      {
        InitializeLayer(kHitStreakOnUngatedSourceLayer);
        std::_Exit(ros::ok() ? EXIT_SUCCESS : EXIT_FAILURE);
      },
      ::testing::ExitedWithCode(EXIT_SUCCESS), kUnusedHitStreakWarning);
}

TEST_F(LayerVoxelClassParamsTest, CreationGatedSourceDoesNotWarnAtStartup) {
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  EXPECT_EXIT(
      {
        ::testing::internal::CaptureStderr();
        InitializeLayer(kCreationGatedSourceLayer);
        const bool warned = ::testing::internal::GetCapturedStderr().find(
                                kUnusedHitStreakWarning) != std::string::npos;
        std::_Exit(ros::ok() && !warned ? EXIT_SUCCESS : EXIT_FAILURE);
      },
      ::testing::ExitedWithCode(EXIT_SUCCESS), "");
}

TEST_F(LayerVoxelClassParamsTest, PublishingClassAdvertisesItsVoxelTopic) {
  InitializeLayer(kClassifiedLayer);
  ASSERT_TRUE(ros::ok());

  EXPECT_TRUE(IsAdvertised(kCliffVoxelTopic, "sensor_msgs/PointCloud2"));
}

TEST_F(LayerVoxelClassParamsTest, ClearClassVoxelsSucceedsOnceInitialized) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeLayer(kClassifiedLayer);
  ASSERT_TRUE(ros::ok());

  const bear_msgs::ClearClassVoxels::Response resp =
      ClearClassVoxels(*layer, "cliff");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_TRUE(resp.message.empty());
}

TEST_F(LayerVoxelClassParamsTest, VehicleClassAdvertisesItsVoxelTopic) {
  InitializeLayer(kClassifiedLayer);
  ASSERT_TRUE(ros::ok());

  EXPECT_TRUE(IsAdvertised(kVehicleVoxelTopic, "sensor_msgs/PointCloud2"));
}

TEST_F(LayerVoxelClassParamsTest, ClearVehicleVoxelsSucceedsOnceInitialized) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeLayer(kClassifiedLayer);
  ASSERT_TRUE(ros::ok());

  const bear_msgs::ClearClassVoxels::Response resp =
      ClearClassVoxels(*layer, "vehicle");

  EXPECT_TRUE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_TRUE(resp.message.empty());
}

TEST_F(LayerVoxelClassParamsTest, ClearClassVoxelsRejectsAnUnconfiguredClass) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeLayer(kGenericOnlyLayer);
  ASSERT_TRUE(ros::ok());

  const bear_msgs::ClearClassVoxels::Response resp =
      ClearClassVoxels(*layer, "cliff");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_NE(resp.message.find("'cliff' is not configured"), std::string::npos)
      << "the rejection must name the class and say why, got: " << resp.message;
}

TEST_F(LayerVoxelClassParamsTest, MapChangeResetRedrawsTheWholeLayer) {
  boost::shared_ptr<SpatioTemporalVoxelLayer> layer =
      InitializeLayer(kClassifiedLayer);
  ASSERT_TRUE(ros::ok());

  layer->activateByMapChange();
  layered_costmap_.updateMap(0.0, 0.0, 0.0);

  EXPECT_TRUE(LastUpdateRedrewTheWholeMap())
      << "a reset must hand the whole cleared area back to the master costmap";
  EXPECT_TRUE(layer->isCurrent());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_layer_voxel_class_params");
  return RUN_ALL_TESTS();
}
