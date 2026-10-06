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
 * Purpose: Implement OpenVDB's voxel library with ray tracing for our
 *          internal voxel grid layer.
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
 *  - Various bug fixes and performance improvements
 *    (see git history for detailed per-commit changes)
 * Contributors:
 *  - Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *  - Shivani Sivakumar (shivani.sivakumar@bearrobotics.ai)
 *  - Hashir Zahir (hashir.zahir@bearrobotics.ai)
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

#ifndef VOLUME_GRID_H_
#define VOLUME_GRID_H_

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
// ROS
#include <ros/ros.h>
// STL
#include <math.h>

#include <ctime>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
// msgs
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/RayIntersector.h>
// measurement struct and buffer
#include <spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp>
#include <spatio_temporal_voxel_layer/measurement_buffer.hpp>
#include <spatio_temporal_voxel_layer/robot_motion_tracker.hpp>
// Mutex and locks
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace volume_grid {

enum GlobalDecayModel { LINEAR = 0, EXPONENTIAL = 1, PERSISTENT = 2 };

// Structure for an occupied cell for map
struct occupany_cell {
  occupany_cell(const double& _x, const double& _y) : x(_x), y(_y) {}

  bool operator==(const occupany_cell& other) const {
    return x == other.x && y == other.y;
  }

  double x, y;
};

// Structure for wrapping frustum model and necessary metadata
struct frustum_model {
  frustum_model(std::unique_ptr<geometry::Frustum> _frustum, double _factor)
      : frustum(std::move(_frustum)), accel_factor(_factor) {}
  std::unique_ptr<geometry::Frustum> frustum;
  const double accel_factor;
};

// Core voxel grid structure and interface
class SpatioTemporalVoxelGrid {
 public:
  // conveniences for line lengths
  typedef openvdb::math::Ray<openvdb::Real> GridRay;
  typedef openvdb::math::Ray<openvdb::Real>::Vec3T Vec3Type;

  SpatioTemporalVoxelGrid(
      const float& voxel_size, const double& background_value,
      const int& decay_model, const double& voxel_decay, const bool& pub_voxels,
      std::unique_ptr<geometry::FootprintFrustum> safety_zone_frustum,
      std::unique_ptr<geometry::InterSensorDecayPrism> inter_sensor_decay_prism,
      std::unique_ptr<geometry::FootprintClearingPrism>
          front_blind_spot_clearing_prism,
      std::unique_ptr<DynamicObstacleTracker> dynamic_obstacle_tracker,
      std::unique_ptr<RobotMotionTracker> robot_motion_tracker);
  ~SpatioTemporalVoxelGrid(void);

  // Core making and clearing functions
  void Mark(
      const std::vector<observation::MeasurementReading>& marking_observations);
  void operator()(const observation::MeasurementReading& obs) const;
  void ClearFrustums(
      const std::vector<observation::MeasurementReading>& clearing_observations,
      std::unordered_set<occupany_cell>& cleared_cells,
      std::vector<DynamicObstacleReading>& dynamic_obstacle_readings,
      std::vector<RobotMotionReading>& robot_motion_readings);

  /**
   * @brief Clears all voxels within a circular area in the XY plane.
   * @details Iterates through all active voxels in the grid and removes any
   * voxels whose XY position falls within the specified circular region.
   * The Z coordinate is ignored - all voxels at any height within the circular
   * footprint are cleared. Thread-safe operation using a mutex lock.
   * @param center_x Center X coordinate of the circular area in the global
   *                 frame (meters).
   * @param center_y Center Y coordinate of the circular area in the global
   *                 frame (meters).
   * @param radius Radius of the circular area to clear (meters).
   */
  void ClearCircularArea(double center_x, double center_y, double radius);

  /**
   * @brief Updates the robot's current pose for frustum transformations.
   * @details Stores the robot's pose which is used by safety zone frustum and
   * dynamic obstacle tracking for coordinate transformations.
   * @param x Robot's x position in the global frame (meters).
   * @param y Robot's y position in the global frame (meters).
   * @param yaw Robot's yaw orientation in the global frame (radians).
   */
  void SetRobotPose(double x, double y, double yaw);

  /**
   * @brief Adds frustum visualization markers for RViz display.
   * @details Assigns sensor-specific colors to frustum markers and publishes
   * them for visualization. Different sensors (e.g., astra_depth,
   * astra_down_depth, astra_up_depth) are assigned unique colors.
   * @param sensor_name Name of the sensor frustum being visualized.
   * @param frustum_marker Marker array containing the frustum geometry.
   */
  void AddVisualizationMarker(
      const std::string& sensor_name,
      const visualization_msgs::MarkerArray& frustum_marker);

  /**
   * @brief Checks if a point is located in the sensor dead zone.
   * @details Determines whether a given 3D point falls within the robot's
   * safety zone frustum, which represents areas not visible to any sensor.
   * @param point The 3D point in world coordinates to check.
   * @return True if the point is inside the safety zone frustum (dead zone),
   *         false otherwise or if no safety zone frustum is configured.
   */
  bool IsObstacleInSensorDeadZone(const openvdb::Vec3d& point) const;

  // Get the pointcloud of the underlying occupancy grid
  void GetOccupancyPointCloud(sensor_msgs::PointCloud2::Ptr& pc2);
  std::unordered_map<occupany_cell, uint>* GetFlattenedCostmap();

  // Clear the grid
  bool ResetGrid(void);
  void ResetGridArea(const occupany_cell& start, const occupany_cell& end,
                     bool invert_area = false);

  // Save the file to file with size information
  bool SaveGrid(const std::string& file_name, double& map_size_bytes);

  /**
   * @brief Updates the stored sensor frustum for a given sensor.
   * @details Maintains a list of the most recent sensor readings (frustums) for
   * each sensor. If the sensor already has a stored reading, it updates only if
   * the new reading is more recent. Otherwise, adds a new entry.
   * @param reading The measurement reading containing sensor name, timestamp,
   *                origin, orientation, and frustum factory.
   */
  void UpdateLastReadings(const observation::MeasurementReading& reading);

  /**
   * @brief Checks if a point is visible within any stored sensor frustum.
   * @details Iterates through all stored sensor frustums and returns true if
   * the point falls inside any of them. Used to determine if an occupied voxel
   * is currently observable or in a blind spot.
   * @param point The 3D point in world coordinates to check.
   * @return True if the point is inside at least one sensor frustum.
   */
  bool IsPointInLastSensorFrustums(const openvdb::Vec3d& point) const;

  /**
   * @brief Finds an occupied voxel in a region that is not visible to sensors.
   * @details Searches for active (occupied) voxels within the specified
   * axis-aligned bounding box in world coordinates. Returns the first voxel
   * found that is not inside any current sensor frustum (i.e., a blind spot).
   * @param min_corner Minimum corner of the search box in world coordinates.
   * @param max_corner Maximum corner of the search box in world coordinates.
   * @return World coordinates of an occupied blind spot voxel, or nullopt if
   *         no blind spot exists in the region.
   */
  std::optional<openvdb::Vec3d> CheckBox(
      const openvdb::Vec3d& min_corner, const openvdb::Vec3d& max_corner) const;

  /**
   * @brief Finds all active voxels at a given XY location within a tolerance.
   * @param x The x coordinate to search at.
   * @param y The y coordinate to search at.
   * @param tolerance The radial distance tolerance for matching voxels.
   * @return A vector of world coordinates of all matching voxels.
   */
  std::vector<openvdb::Vec3d> GetVoxelsAtXY(double x, double y,
                                            double tolerance) const;

 protected:
  // Initialize grid metadata and library
  void InitializeGrid(void);

  // grid accessor methods
  bool MarkGridPoint(const openvdb::Coord& pt, double value,
                     openvdb::DoubleGrid::Accessor& value_accessor) const;
  bool ClearGridPoint(const openvdb::Coord& pt,
                      openvdb::DoubleGrid::Accessor& value_accessor) const;

  // Check occupancy status of the grid
  bool IsGridEmpty(void) const;

  // Get time information for clearing
  double GetTemporalClearingDuration(const double& time_delta);
  double GetFrustumAcceleration(const double& time_delta,
                                const double& acceleration_factor);
  void TemporalClearAndGenerateCostmap(
      std::vector<frustum_model>& frustums,
      std::unordered_set<occupany_cell>& cleared_cells,
      std::vector<std::unique_ptr<geometry::IClearingFrustum>>&
          dynamic_obstacle_frustums);

  // Populate the costmap ROS api and pointcloud with a marked point
  void PopulateCostmapAndPointcloud(const openvdb::Vec3d& pose_world);

  // Utilities for tranformation
  openvdb::Vec3d WorldToIndex(const openvdb::Vec3d& coord) const;
  openvdb::Vec3d IndexToWorld(const openvdb::Coord& coord) const;

  struct LastReading {
    ros::Time time;
    std::string sensor_name;
    std::unique_ptr<geometry::Frustum> frustum;
  };
  std::vector<LastReading> last_readings_;

  mutable openvdb::DoubleGrid::Ptr _grid;
  int _decay_model;
  double _background_value, _voxel_size, _voxel_decay;
  bool _pub_voxels;
  std::vector<geometry_msgs::Point32>* _grid_points;
  std::unordered_map<occupany_cell, uint>* _cost_map;
  boost::mutex _grid_lock;
  std::unique_ptr<geometry::FootprintFrustum> _safety_zone_frustum;
  std::unique_ptr<geometry::InterSensorDecayPrism> _inter_sensor_decay_prism;
  std::unique_ptr<geometry::FootprintClearingPrism>
      _front_blind_spot_clearing_prism;
  std::unique_ptr<DynamicObstacleTracker> _dynamic_obstacle_tracker;
  std::unique_ptr<RobotMotionTracker> _robot_motion_tracker;
  ros::NodeHandle _nh;
  ros::Publisher _frustum_viz_pub;
  double _robot_x;
  double _robot_y;
  double _robot_yaw;
  visualization_msgs::MarkerArray _frustum_markers;
};

}  // namespace volume_grid

// hash function for unordered_map of occupancy_cells
namespace std {
template <>
struct hash<volume_grid::occupany_cell> {
  std::size_t operator()(const volume_grid::occupany_cell& k) const {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1);
  }
};

}  // namespace std

#endif
