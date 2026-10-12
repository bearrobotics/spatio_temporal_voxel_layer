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

#include <openvdb/openvdb.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using test_utils::TestableGrid;

std::unique_ptr<TestableGrid> MakeGrid() {
  openvdb::initialize();
  return std::make_unique<TestableGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue,
      test_utils::kDecayModel, test_utils::kVoxelDecay, test_utils::kPubVoxels,
      test_utils::MakeMockFootprintFrustum(),
      test_utils::MakeMockInterSensorDecayPrism(),
      /*front_blind_spot_clearing_prism=*/nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker(), std::nullopt);
}

visualization_msgs::MarkerArray MakeMarkers(std::size_t count) {
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(count);
  return markers;
}

bool IsSameColor(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

TEST(AddVisualizationMarkerTest, TwoSourcesGetDistinctNamespacesAndColors) {
  std::unique_ptr<TestableGrid> grid = MakeGrid();

  grid->AddVisualizationMarker("gemini_down_marking", MakeMarkers(2));
  grid->AddVisualizationMarker("gemini_down_cliff", MakeMarkers(2));

  const std::vector<visualization_msgs::Marker>& markers =
      grid->GetFrustumMarkers().markers;
  ASSERT_EQ(markers.size(), 4u);
  EXPECT_EQ(markers[0].ns, "gemini_down_marking");
  EXPECT_EQ(markers[1].ns, "gemini_down_marking");
  EXPECT_EQ(markers[2].ns, "gemini_down_cliff");
  EXPECT_EQ(markers[3].ns, "gemini_down_cliff");
  EXPECT_FALSE(IsSameColor(markers[0].color, markers[2].color));
}

TEST(AddVisualizationMarkerTest, MarkerIdsCountUpWithinOneSource) {
  std::unique_ptr<TestableGrid> grid = MakeGrid();

  grid->AddVisualizationMarker("gemini_down_marking", MakeMarkers(3));

  // rviz overwrites markers that share a namespace and id.
  const std::vector<visualization_msgs::Marker>& markers =
      grid->GetFrustumMarkers().markers;
  ASSERT_EQ(markers.size(), 3u);
  EXPECT_EQ(markers[0].id, 0);
  EXPECT_EQ(markers[1].id, 1);
  EXPECT_EQ(markers[2].id, 2);
}

TEST(AddVisualizationMarkerTest, RepeatedSourceKeepsItsFirstColor) {
  std::unique_ptr<TestableGrid> grid = MakeGrid();

  grid->AddVisualizationMarker("gemini_down_cliff", MakeMarkers(1));
  grid->AddVisualizationMarker("gemini_down_marking", MakeMarkers(1));
  grid->AddVisualizationMarker("gemini_down_cliff", MakeMarkers(1));

  const std::vector<visualization_msgs::Marker>& markers =
      grid->GetFrustumMarkers().markers;
  ASSERT_EQ(markers.size(), 3u);
  EXPECT_TRUE(IsSameColor(markers[0].color, markers[2].color));
}

TEST(AddVisualizationMarkerTest, MoreSourcesThanPaletteReuseColors) {
  std::unique_ptr<TestableGrid> grid = MakeGrid();

  constexpr int kSourceCount = 20;
  for (int i = 0; i < kSourceCount; ++i) {
    grid->AddVisualizationMarker("source_" + std::to_string(i), MakeMarkers(1));
  }

  // Sources past the palette end must wrap to an already used color.
  const std::vector<visualization_msgs::Marker>& markers =
      grid->GetFrustumMarkers().markers;
  ASSERT_EQ(markers.size(), static_cast<std::size_t>(kSourceCount));
  std::set<std::tuple<float, float, float>> distinct_colors;
  for (const visualization_msgs::Marker& marker : markers) {
    EXPECT_GT(marker.color.a, 0.0f);
    distinct_colors.insert({marker.color.r, marker.color.g, marker.color.b});
  }
  EXPECT_LT(distinct_colors.size(), static_cast<std::size_t>(kSourceCount));
  EXPECT_GT(distinct_colors.size(), 1u);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_frustum_visualization");
  return RUN_ALL_TESTS();
}
