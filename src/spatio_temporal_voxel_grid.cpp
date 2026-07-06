/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *********************************************************************/
/*
 * --- BEAR MODIFICATION START ---
 * Portions Copyright (c) 2025-2026, Bear Robotics, Inc.
 * This file was modified by Bear Robotics, Inc. between 2025 and 2026.
 * Description of changes:
 *  - Integration with Bear Robotics internal navigation stack
 *  - Multi-robot obstacle tracking and coordination support
 *  - Robot motion tracking for self-clearing
 *  - Front blind-spot clearing prism for near-range obstacle clearing
 *  - Inter-sensor decay prism that accelerates voxel decay in the
 *    blind-spot region between sensors
 *  - CheckBlindSpot and ClearRobotFootprint services
 *  - Sensor data filtering (noise filter, frustum-based filtering)
 *  - Safety zone frustum support
 *  - Permanent cliff voxels (parallel per-voxel class grid; never decay or
 *    clear) with a ClearCliffs operation
 *  - Various bug fixes and performance improvements
 *    (see git history for detailed per-commit changes)
 * Contributors:
 *  - Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *  - Shivani Sivakumar (shivani.sivakumar@bearrobotics.ai)
 *  - Seung-Hun (Hoon) Han (seunghun.han@bearrobotics.ai)
 * --- BEAR MODIFICATION END ---
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp>

namespace volume_grid {

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(
    const float& voxel_size, const double& background_value,
    const int& decay_model, const double& voxel_decay, const bool& pub_voxels,
    std::unique_ptr<geometry::FootprintFrustum> safety_zone_frustum,
    std::unique_ptr<geometry::InterSensorDecayPrism> inter_sensor_decay_prism,
    std::unique_ptr<geometry::FootprintClearingPrism>
        front_blind_spot_clearing_prism,
    std::unique_ptr<DynamicObstacleTracker> dynamic_obstacle_tracker,
    std::unique_ptr<RobotMotionTracker> robot_motion_tracker)
    : _background_value(background_value),
      _voxel_size(voxel_size),
      _decay_model(decay_model),
      _voxel_decay(voxel_decay),
      _pub_voxels(pub_voxels),
      _safety_zone_frustum(std::move(safety_zone_frustum)),
      _inter_sensor_decay_prism(std::move(inter_sensor_decay_prism)),
      _front_blind_spot_clearing_prism(
          std::move(front_blind_spot_clearing_prism)),
      _dynamic_obstacle_tracker(std::move(dynamic_obstacle_tracker)),
      _robot_motion_tracker(std::move(robot_motion_tracker)),
      _grid_points(new std::vector<geometry_msgs::Point32>),
      _cost_map(new std::unordered_map<occupany_cell, uint>),
      _nh()
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  // pcl pointclouds free themselves
  if (_cost_map) {
    delete _cost_map;
  }

  if (_grid_points) {
    delete _grid_points;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  // make it default to background value
  _grid = openvdb::DoubleGrid::create(_background_value);

  // setup scale and tranform
  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  // setup transform and other metadata
  _grid->setTransform(openvdb::math::Transform::createLinearTransform(m));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);

  _class_grid = openvdb::Int32Grid::create(kGeneric);
  _class_grid->setTransform(_grid->transform().copy());
  _class_grid->setName("SpatioTemporalVoxelClass");

  _frustum_viz_pub = _nh.advertise<visualization_msgs::MarkerArray>(
      "/spatio_temporal_voxel_layer/frustums", 1);
  return;
}

void SpatioTemporalVoxelGrid::SetRobotPose(double x, double y, double yaw) {
  _robot_x = x;
  _robot_y = y;
  _robot_yaw = yaw;
}

void SpatioTemporalVoxelGrid::ClearCircularArea(double center_x,
                                                double center_y,
                                                double radius) {
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  const double radius_sq = radius * radius;
  openvdb::Int32Grid::ConstAccessor class_accessor =
      _class_grid->getConstAccessor();
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    // Footprint clearing must not erase permanent cliffs.
    if (class_accessor.getValue(pt_index) == kCliff) {
      continue;
    }
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    const double dx = pose_world.x() - center_x;
    const double dy = pose_world.y() - center_y;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq <= radius_sq) {
      ClearGridPoint(pt_index);
    }
  }
}

void SpatioTemporalVoxelGrid::ClearCliffs(void) {
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  openvdb::Int32Grid::ConstAccessor class_accessor =
      _class_grid->getConstAccessor();
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    if (class_accessor.getValue(pt_index) == kCliff) {
      ClearGridPoint(pt_index);
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums(
    const std::vector<observation::MeasurementReading>& clearing_readings,
    std::unordered_set<occupany_cell>& cleared_cells,
    std::vector<DynamicObstacleReading>& dynamic_obstacle_readings,
    std::vector<RobotMotionReading>& robot_motion_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // Keep robot-anchored debug geometry current even when no voxels are active.
  geometry_msgs::Point robot_position;
  robot_position.x = _robot_x;
  robot_position.y = _robot_y;
  robot_position.z = 0.0;
  tf2::Quaternion q;
  // roll=0, pitch=0, yaw=yaw
  q.setRPY(0, 0, _robot_yaw);
  geometry_msgs::Quaternion robot_orientation = tf2::toMsg(q);
  _safety_zone_frustum->SetPosition(robot_position);
  _safety_zone_frustum->SetOrientation(robot_orientation);
  _safety_zone_frustum->TransformModel();
  _safety_zone_frustum->PublishVisualization();
  if (_inter_sensor_decay_prism) {
    _inter_sensor_decay_prism->SetPosition(robot_position);
    _inter_sensor_decay_prism->SetOrientation(robot_orientation);
    _inter_sensor_decay_prism->TransformModel();
    _inter_sensor_decay_prism->PublishVisualization();
  }
  if (_front_blind_spot_clearing_prism) {
    _front_blind_spot_clearing_prism->SetPosition(robot_position);
    _front_blind_spot_clearing_prism->SetOrientation(robot_orientation);
    _front_blind_spot_clearing_prism->TransformModel();
    _front_blind_spot_clearing_prism->PublishVisualization();
  }

  // accelerate the decay of voxels interior to the frustum
  if (this->IsGridEmpty()) {
    _grid_points->clear();
    _cost_map->clear();
    return;
  }

  _grid_points->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;
  std::vector<std::unique_ptr<geometry::IClearingFrustum>>
      dynamic_obstacle_frustums;
  dynamic_obstacle_frustums =
      _dynamic_obstacle_tracker->GenerateDynamicObstacleClearingFrustums(
          dynamic_obstacle_readings);
  auto robot_motion_frustums =
      _robot_motion_tracker->GenerateRobotMotionClearingFrustums(
          robot_motion_readings);
  dynamic_obstacle_frustums.insert(
      dynamic_obstacle_frustums.end(),
      std::make_move_iterator(robot_motion_frustums.begin()),
      std::make_move_iterator(robot_motion_frustums.end()));
  if (dynamic_obstacle_frustums.size() > 0) {
    ROS_WARN_THROTTLE(10, "Number of circles: %d",
                      dynamic_obstacle_frustums.size());
  }
  _frustum_markers.markers.clear();
  if (!clearing_readings.empty()) {
    obs_frustums.reserve(clearing_readings.size());

    for (const observation::MeasurementReading& reading : clearing_readings) {
      std::unique_ptr<geometry::Frustum> frustum =
          (reading._frustrum_factory)();
      frustum->SetPosition(reading._origin);
      frustum->SetOrientation(reading._orientation);
      frustum->TransformModel();

      visualization_msgs::MarkerArray frustum_marker;
      frustum->GetVisualizationMarker(frustum_marker);
      AddVisualizationMarker(reading._sensor_name, frustum_marker);
      obs_frustums.emplace_back(std::move(frustum),
                                reading._decay_acceleration);
      UpdateLastReadings(reading);
    }
    if (!_frustum_markers.markers.empty()) {
      _frustum_viz_pub.publish(_frustum_markers);
    }
  }
  TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells,
                                  dynamic_obstacle_frustums);
  return;
}

void SpatioTemporalVoxelGrid::AddVisualizationMarker(
    const std::string& sensor_name,
    const visualization_msgs::MarkerArray& frustum_marker) {
  std_msgs::ColorRGBA color;
  if (sensor_name.find("astra_depth_optical_frame") != std::string::npos) {
    color.r = 0.0f;
    color.g = 0.0f;
    color.b = 1.0f;
    color.a = 0.5f;
  } else if (sensor_name.find("astra_down_depth_optical_frame") !=
             std::string::npos) {
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.0f;
    color.a = 0.5f;
  } else if (sensor_name.find("astra_up_depth_optical_frame") !=
             std::string::npos) {
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 0.5f;
  } else {
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 1.0f;
    color.a = 0.5f;
  }
  for (const auto& marker : frustum_marker.markers) {
    visualization_msgs::Marker mod_marker = marker;
    mod_marker.ns = sensor_name + "_" + marker.ns;
    mod_marker.color = color;
    _frustum_markers.markers.push_back(mod_marker);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(
    std::vector<frustum_model>& frustums,
    std::unordered_set<occupany_cell>& cleared_cells,
    std::vector<std::unique_ptr<geometry::IClearingFrustum>>&
        dynamic_obstacle_frustums)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  const double cur_time = ros::Time::now().toSec();

  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  openvdb::Int32Grid::ConstAccessor class_accessor =
      _class_grid->getConstAccessor();
  for (cit_grid; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());

    // CLIFF voxels are permanent: never decay, regardless of decay model or
    // age.
    if (class_accessor.getValue(pt_index) == kCliff) {
      PopulateCostmapAndPointcloud(pt_index);
      continue;
    }

    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;
    bool cleared_point = false;

    const double time_since_marking = cur_time - cit_grid.getValue();
    const double base_duration_to_decay =
        GetTemporalClearingDuration(time_since_marking);

    for (frustum_it; frustum_it != frustums.end(); ++frustum_it) {
      if (!frustum_it->frustum) {
        continue;
      }

      if (frustum_it->frustum->IsInside(pose_world)) {
        frustum_cycle = true;

        const double frustum_acceleration = GetFrustumAcceleration(
            time_since_marking, frustum_it->accel_factor);

        const double time_until_decay =
            base_duration_to_decay - frustum_acceleration;
        if (time_until_decay < 0.) {
          // expired by acceleration
          cleared_point = true;
          if (!this->ClearGridPoint(pt_index)) {
            ROS_WARN_THROTTLE(5.0, "Failed to clear point.");
          }
          break;
        } else {
          const double updated_mark =
              cit_grid.getValue() - frustum_acceleration;
          if (!this->MarkGridPoint(pt_index, updated_mark)) {
            ROS_WARN("Failed to update mark.");
          }
          break;
        }
      }
    }

    // Check if the point is in a dynamic obstacle clearing frustum
    for (const std::unique_ptr<geometry::IClearingFrustum>& frustum :
         dynamic_obstacle_frustums) {
      if (frustum->IsInside(pose_world)) {
        ROS_WARN_THROTTLE(10, "CLEARED DYNAMIC POINT:Point: [%f, %f, %f]",
                          pose_world[0], pose_world[1], pose_world[2]);
        frustum_cycle = true;
        cleared_point = true;
        if (!this->ClearGridPoint(pt_index)) {
          ROS_WARN_THROTTLE(5.0, "Failed to clear point.");
        }
        break;
      }
    }

    // Accelerate decay for voxels in the inter-sensor blind spot
    if (!cleared_point && !frustum_cycle && _inter_sensor_decay_prism &&
        _inter_sensor_decay_prism->IsInside(pose_world)) {
      frustum_cycle = true;

      const double inter_sensor_acceleration = GetFrustumAcceleration(
          time_since_marking,
          _inter_sensor_decay_prism->decay_acceleration_factor());

      const double time_until_decay =
          base_duration_to_decay - inter_sensor_acceleration;
      if (time_until_decay < 0.) {
        cleared_point = true;
        if (!this->ClearGridPoint(pt_index)) {
          ROS_WARN("Failed to clear point.");
        }
      } else {
        const double updated_mark =
            cit_grid.getValue() - inter_sensor_acceleration;
        if (!this->MarkGridPoint(pt_index, updated_mark)) {
          ROS_WARN("Failed to update mark.");
        }
      }
    }

    if (!cleared_point && _front_blind_spot_clearing_prism &&
        _front_blind_spot_clearing_prism->IsInside(pose_world)) {
      frustum_cycle = true;
      cleared_point = true;
      if (!this->ClearGridPoint(pt_index)) {
        ROS_WARN_THROTTLE(5.0, "Failed to clear point.");
      }
    }

    // if not inside any, check against nominal decay model
    if (!frustum_cycle) {
      if (IsObstacleInSensorDeadZone(pose_world)) {
        ROS_WARN_THROTTLE(5.,
                          "Obstacle in sensor dead zone. Point: [%f, %f, %f]",
                          pose_world[0], pose_world[1], pose_world[2]);
      } else if (base_duration_to_decay < 0.) {
        // expired by temporal clearing
        cleared_point = true;
        if (!this->ClearGridPoint(pt_index)) {
          ROS_WARN_THROTTLE(5.0, "Failed to clear point.");
        }
      }
    }

    if (cleared_point) {
      cleared_cells.insert(occupany_cell(pose_world[0], pose_world[1]));
    } else {
      // if here, we can add to costmap and PC2
      PopulateCostmapAndPointcloud(pt_index);
    }
  }

  // free memory taken by expired voxels
  _grid->pruneGrid();
  _class_grid->pruneGrid();
}

/*****************************************************************************/
std::vector<openvdb::Vec3d> SpatioTemporalVoxelGrid::GetVoxelsAtXY(
    double x, double y, double tolerance) const
/*****************************************************************************/
{
  double squared_tolerance = tolerance * tolerance;
  std::vector<openvdb::Vec3d> voxels_at_xy;

  // Iterate over all active voxels in the grid
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d voxel_world = this->IndexToWorld(pt_index);

    // Check if this voxel is at the query XY location (within tolerance)
    const double dx = voxel_world.x() - x;
    const double dy = voxel_world.y() - y;
    const double squared_xy_dist = dx * dx + dy * dy;

    if (squared_xy_dist <= squared_tolerance) {
      voxels_at_xy.push_back(voxel_world);
    }
  }

  return voxels_at_xy;
}

bool SpatioTemporalVoxelGrid::IsPointInLastSensorFrustums(
    const openvdb::Vec3d& pose) const {
  for (const LastReading& reading : last_readings_) {
    if (reading.frustum->IsInside(pose)) {
      return true;
    }
  }
  return false;
}

std::optional<openvdb::Vec3d> SpatioTemporalVoxelGrid::CheckBox(
    const openvdb::Vec3d& min_corner, const openvdb::Vec3d& max_corner) const {
  // 3. Convert World BBox to Index BBox
  // This function handles the complex transformation, including non-uniform
  // scale/rotation. We use worldToIndexCellCentered to ensure the resulting
  // CoordBBox covers all relevant *voxel centers* inside the world space box.
  auto transform = _grid->transform();

  openvdb::Coord index_min = transform.worldToIndexCellCentered(min_corner);
  openvdb::Coord index_max = transform.worldToIndexCellCentered(max_corner);
  openvdb::CoordBBox index_bbox(index_min, index_max);

  // 4. Iterate over Active Voxels in the Index BBox

  // The iterator is constrained by the calculated index_bbox
  for (openvdb::DoubleGrid::ValueOnCIter iter = _grid->cbeginValueOn(); iter;
       ++iter) {
    if (!index_bbox.isInside(iter.getCoord())) {
      continue;
    }
    // Convert the index back to world space for output
    openvdb::Vec3d active_voxel_in_box =
        transform.indexToWorld(iter.getCoord());
    if (_front_blind_spot_clearing_prism &&
        _front_blind_spot_clearing_prism->IsInside(active_voxel_in_box)) {
      continue;
    }
    if (IsPointInLastSensorFrustums(active_voxel_in_box)) {
      continue;
    }
    return active_voxel_in_box;
  }
  return std::nullopt;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::PopulateCostmapAndPointcloud(
    const openvdb::Coord& pt)
/*****************************************************************************/
{
  // add pt to the pointcloud and costmap
  openvdb::Vec3d pose_world = this->IndexToWorld(pt);

  if (_pub_voxels) {
    geometry_msgs::Point32 point;
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
    _grid_points->push_back(point);
  }

  std::unordered_map<occupany_cell, uint>::iterator cell;
  cell = _cost_map->find(occupany_cell(pose_world[0], pose_world[1]));
  if (cell != _cost_map->end()) {
    cell->second += 1;
  } else {
    _cost_map->insert(
        std::make_pair(occupany_cell(pose_world[0], pose_world[1]), 1));
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::Mark(
    const std::vector<observation::MeasurementReading>& marking_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // mark the grid
  if (marking_readings.size() > 0) {
    // tbb::parallel_do(marking_readings, *this); /*must do via merged trees*/
    for (int i = 0; i != marking_readings.size(); i++) {
      (*this)(marking_readings.at(i));
    }
  }
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(
    const observation::MeasurementReading& obs) const
/*****************************************************************************/
{
  if (obs._marking) {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = ros::Time::now().toSec();

    const sensor_msgs::PointCloud2& cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (iter_x, iter_y, iter_z; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z) {
      float distance_2 = (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) +
                         (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) +
                         (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001) {
        continue;
      }

      double x = *iter_x < 0 ? *iter_x - _voxel_size : *iter_x;
      double y = *iter_y < 0 ? *iter_y - _voxel_size : *iter_y;
      double z = *iter_z < 0 ? *iter_z - _voxel_size : *iter_z;

      openvdb::Vec3d mark_grid(this->WorldToIndex(openvdb::Vec3d(x, y, z)));

      if (!this->MarkGridPoint(
              openvdb::Coord(mark_grid[0], mark_grid[1], mark_grid[2]),
              cur_time, obs._voxel_class)) {
        ROS_WARN("Failed to mark point.");
      }
    }
  }
  return;
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint>*
SpatioTemporalVoxelGrid::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(
    const double& time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0)  // linear
  {
    return _voxel_decay - time_delta;
  } else if (_decay_model == 1)  // exponential
  {
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay;  // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration(
    const double& time_delta, const double& acceleration_factor)
/*****************************************************************************/
{
  const double acceleration =
      1. / 6. * acceleration_factor * (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud(
    sensor_msgs::PointCloud2::Ptr& pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  for (std::vector<geometry_msgs::Point32>::iterator it = _grid_points->begin();
       it != _grid_points->end(); ++it) {
    const geometry_msgs::Point32& pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return;
}

/*****************************************************************************/
size_t SpatioTemporalVoxelGrid::GetCliffPointCloud(
    sensor_msgs::PointCloud2::Ptr& pc2)
/*****************************************************************************/
{
  _cliff_points.clear();
  openvdb::Int32Grid::ConstAccessor class_accessor =
      _class_grid->getConstAccessor();
  for (openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
       cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    if (class_accessor.getValue(pt_index) != kCliff) {
      continue;
    }
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);
    geometry_msgs::Point32& point = _cliff_points.emplace_back();
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
  }

  sensor_msgs::PointCloud2Modifier modifier(*pc2);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(_cliff_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");
  for (const geometry_msgs::Point32& point : _cliff_points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return _cliff_points.size();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // clear the voxel grid
  try {
    _grid->clear();
    _class_grid->clear();
    if (this->IsGridEmpty()) {
      return true;
    }
  } catch (...) {
    ROS_WARN("Failed to reset costmap, please try again.");
  }
  return false;
}

/*****************************************************************************************************************/
void SpatioTemporalVoxelGrid::ResetGridArea(const occupany_cell& start,
                                            const occupany_cell& end,
                                            bool invert_area)
/*****************************************************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  openvdb::Int32Grid::ConstAccessor class_accessor =
      _class_grid->getConstAccessor();
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (cit_grid; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    if (class_accessor.getValue(pt_index) == kCliff) {
      continue;
    }
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    const bool in_x_range = pose_world.x() > start.x && pose_world.x() < end.x;
    const bool in_y_range = pose_world.y() > start.y && pose_world.y() < end.y;
    const bool in_range = in_x_range && in_y_range;

    if (in_range == invert_area) {
      ClearGridPoint(pt_index);
    }
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::MarkGridPoint(const openvdb::Coord& pt,
                                            const double& value) const
/*****************************************************************************/
{
  // marking the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::MarkGridPoint(const openvdb::Coord& pt,
                                            double value, int voxel_class) const
/*****************************************************************************/
{
  openvdb::Int32Grid::Accessor class_accessor = _class_grid->getAccessor();
  // CLIFF is permanent; a lower-priority class must never overwrite it.
  if (class_accessor.getValue(pt) != kCliff) {
    class_accessor.setValueOn(pt, voxel_class);
  }
  return this->MarkGridPoint(pt, value);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ClearGridPoint(const openvdb::Coord& pt) const
/*****************************************************************************/
{
  // clearing the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  if (accessor.isValueOn(pt)) {
    accessor.setValueOff(pt, _background_value);
  }

  openvdb::Int32Grid::Accessor class_accessor = _class_grid->getAccessor();
  if (class_accessor.isValueOn(pt)) {
    class_accessor.setValueOff(pt, kGeneric);
  }

  return !accessor.isValueOn(pt);
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::IndexToWorld(
    const openvdb::Coord& coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  openvdb::Vec3d pose_world = _grid->indexToWorld(coord);

  // Using the center for world coordinate
  const double& center_offset = _voxel_size / 2.0;
  pose_world[0] += center_offset;
  pose_world[1] += center_offset;
  pose_world[2] += center_offset;

  return pose_world;
}

void SpatioTemporalVoxelGrid::UpdateLastReadings(
    const observation::MeasurementReading& reading) {
  // First iterate thorough last readings to see if this sensor already has a
  // reading (uses sensor name)
  for (auto& last_reading : last_readings_) {
    if (last_reading.sensor_name != reading._sensor_name) {
      continue;
    }
    if (last_reading.time >= reading._cloud->header.stamp) {
      return;
    }
    // If found, update it if the time is newer
    last_reading.time = reading._cloud->header.stamp;
    last_reading.frustum = reading._frustrum_factory();
    last_reading.frustum->SetPosition(reading._origin);
    last_reading.frustum->SetOrientation(reading._orientation);
    last_reading.frustum->TransformModel();
    return;
  }

  // If not found, add it to the list
  last_readings_.push_back(LastReading{.time = reading._cloud->header.stamp,
                                       .sensor_name = reading._sensor_name,
                                       .frustum = reading._frustrum_factory()});
  last_readings_.back().frustum->SetPosition(reading._origin);
  last_readings_.back().frustum->SetOrientation(reading._orientation);
  last_readings_.back().frustum->TransformModel();
  return;
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::WorldToIndex(
    const openvdb::Vec3d& vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsObstacleInSensorDeadZone(
    const openvdb::Vec3d& point) const
/*****************************************************************************/
{
  if (!_safety_zone_frustum) {
    return false;
  }
  return _safety_zone_frustum->IsInside(point);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::SaveGrid(const std::string& file_name,
                                       double& map_size_bytes)
/*****************************************************************************/
{
  try {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = {_grid};
    file.write(grids);
    file.close();
    map_size_bytes = _grid->memUsage();
    return true;
  } catch (...) {
    map_size_bytes = 0.;
    return false;
  }
  return false;  // best offense is a good defense
}

};  // namespace volume_grid
