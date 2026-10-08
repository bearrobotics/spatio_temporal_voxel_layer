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
#include <openvdb/openvdb.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
namespace spatio_temporal_voxel_layer::test_utils {
using geometry::FootprintClearingPrism;
using geometry::FootprintFrustum;
using geometry::InterSensorDecayPrism;
using volume_grid::SpatioTemporalVoxelGrid;
using volume_grid::VoxelClass;

class TestableGrid : public SpatioTemporalVoxelGrid {
 public:
  using SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid;

  // Not Grid::empty(), which stays false until a cleared voxel is pruned.
  [[nodiscard]] bool IsClassGridEmpty() const {
    return _class_grid->activeVoxelCount() == 0;
  }

  // The cloud stores float, so round the same way before indexing.
  [[nodiscard]] openvdb::Coord GetCoordAt(
      const geometry_msgs::Point& point) const {
    const openvdb::Vec3d index =
        WorldToIndex({static_cast<float>(point.x), static_cast<float>(point.y),
                      static_cast<float>(point.z)});
    return openvdb::Coord(index[0], index[1], index[2]);
  }

  [[nodiscard]] VoxelClass GetClassAt(const geometry_msgs::Point& point) const {
    return volume_grid::ToVoxelClass(
        _class_grid->getAccessor().getValue(GetCoordAt(point)));
  }

  // Rewinds a mark time so a pass sees an old mark without waiting.
  void AgeMark(const geometry_msgs::Point& point, double seconds) {
    const openvdb::Coord coord = GetCoordAt(point);
    openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();
    accessor.setValueOn(coord, accessor.getValue(coord) - seconds);
  }

  [[nodiscard]] std::size_t GetPendingPromotionCount() const {
    return _promotion_candidates.size();
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

// Mock object creators - create disabled instances for testing
std::unique_ptr<FootprintFrustum> MakeMockFootprintFrustum();
std::unique_ptr<DynamicObstacleTracker> MakeMockDynamicObstacleTracker();
std::unique_ptr<RobotMotionTracker> MakeMockRobotMotionTracker();
std::unique_ptr<InterSensorDecayPrism> MakeMockInterSensorDecayPrism();

// Test grid creator - creates a SpatioTemporalVoxelGrid with mock dependencies
std::unique_ptr<SpatioTemporalVoxelGrid> MakeTestGrid(
    std::unique_ptr<InterSensorDecayPrism> inter_sensor_decay_prism = nullptr,
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
