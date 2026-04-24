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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <vector>

#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
namespace spatio_temporal_voxel_layer::test_utils {
using geometry::FootprintClearingPrism;
using geometry::FootprintFrustum;
using volume_grid::SpatioTemporalVoxelGrid;
// Mock object creators - create disabled instances for testing
std::unique_ptr<FootprintFrustum> MakeMockFootprintFrustum();
std::unique_ptr<DynamicObstacleTracker> MakeMockDynamicObstacleTracker();
std::unique_ptr<RobotMotionTracker> MakeMockRobotMotionTracker();

// Test grid creator - creates a SpatioTemporalVoxelGrid with mock dependencies
std::unique_ptr<SpatioTemporalVoxelGrid> MakeTestGrid(
    std::unique_ptr<FootprintClearingPrism> front_blind_spot_clearing_prism =
        nullptr);

// Test data helpers
sensor_msgs::PointCloud2 MakePointCloud(
    const std::vector<geometry_msgs::Point>& points);

// Test constants
constexpr float kVoxelSize = 0.05f;
constexpr double kBackgroundValue = 0.0;
constexpr int kDecayModel = 2;  // volume_grid::PERSISTENT
constexpr double kVoxelDecay = 0.0;
constexpr bool kPubVoxels = false;

}  // namespace spatio_temporal_voxel_layer::test_utils

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_
