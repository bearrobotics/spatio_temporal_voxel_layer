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
 * Purpose: Replace the ROS voxel grid / obstacle layers using VoxelGrid
 *          with OpenVDB's more efficient and capacble voxel library with
 *          ray tracing and knn.
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
 *  - CheckBlindSpot and ClearRobotFootprint services
 *  - Sensor data filtering (noise filter, frustum-based filtering)
 *  - Safety zone frustum support
 *  - Various bug fixes and performance improvements
 *    (see git history for detailed per-commit changes)
 * Contributors:
 *  - Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *  - Shivani Sivakumar (shivani.sivakumar@bearrobotics.ai)
 *  - Hashir Zahir (hashir.zahir@bearrobotics.ai)
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

#ifndef VOLUME_GRID_LAYER_H_
#define VOLUME_GRID_LAYER_H_

// voxel grid
#include <spatio_temporal_voxel_layer/SpatioTemporalVoxelLayerConfig.h>

#include <spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp>
// ROS
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
// costmap
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
// openVDB
#include <openvdb/openvdb.h>
// STL
#include <time.h>

#include <iostream>
#include <string>
#include <vector>
// msgs
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <spatio_temporal_voxel_layer/CheckBlindSpot.h>
#include <spatio_temporal_voxel_layer/SaveGrid.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
// projector
#include <laser_geometry/laser_geometry.h>
// tf
#include <tf2/buffer_core.h>

#include <mutex>

#include "message_filters/subscriber.h"
#include "multi_robot_public/RobotMotions.h"
#include "obstacle_detector/Obstacles.h"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
namespace spatio_temporal_voxel_layer {

// conveniences for line lengths
typedef std::vector<boost::shared_ptr<message_filters::SubscriberBase> >::
    iterator observation_subscribers_iter;
typedef std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >::iterator
    observation_buffers_iter;
typedef spatio_temporal_voxel_layer::SpatioTemporalVoxelLayerConfig
    dynamicReconfigureType;
typedef dynamic_reconfigure::Server<dynamicReconfigureType>
    dynamicReconfigureServerType;

// Core ROS voxel layer class
class SpatioTemporalVoxelLayer : public costmap_2d::CostmapLayer {
 public:
  SpatioTemporalVoxelLayer(void);
  virtual ~SpatioTemporalVoxelLayer(void);

  // Core Functions
  virtual void onInitialize(void);
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

  // Functions to interact with other layers
  virtual void matchSize(void);

  // Functions for layer high level operations
  virtual void reset(void);
  virtual void activate(void);
  virtual void deactivate(void);

  // Functions for sensor feeds
  bool GetMarkingObservations(
      std::vector<observation::MeasurementReading>& marking_observations) const;
  bool GetClearingObservations(
      std::vector<observation::MeasurementReading>& marking_observations) const;
  void ObservationsResetAfterReading() const;

  /**
   * @brief Retrieves dynamic obstacle readings from the buffer.
   * @details Extracts all buffered dynamic obstacle readings and clears the
   * internal buffer. Thread-safe operation using a mutex lock.
   * @param[out] dynamic_obstacle_readings Vector to populate with current
   *                                       dynamic obstacle readings.
   * @return Always returns true to indicate readings were successfully
   *         retrieved.
   */
  bool GetDynamicObstacleReadings(
      std::vector<DynamicObstacleReading>& dynamic_obstacle_readings);

  /**
   * @brief Retrieves robot motion readings from the buffer.
   * @details Extracts all buffered robot motion readings and clears the
   * internal buffer. Thread-safe operation using a mutex lock.
   * @param[out] robot_motion_readings Vector to populate with current robot
   *                                   motion readings.
   * @return Always returns true to indicate readings were successfully
   *         retrieved.
   */
  bool GetRobotMotionReadings(
      std::vector<RobotMotionReading>& robot_motion_readings);

  // Functions to interact with maps
  void UpdateROSCostmap(
      double* min_x, double* min_y, double* max_x, double* max_y,
      std::unordered_set<volume_grid::occupany_cell>& cleared_cells);
  bool updateFootprint(double robot_x, double robot_y, double robot_yaw,
                       double* min_x, double* min_y, double* max_x,
                       double* max_y);

  /**
   * @brief Resets the underlying OpenVDB voxel grid.
   * @details Clears the level set in the spatio-temporal voxel grid, removing
   * all stored voxel data. Logs a warning if the reset operation fails.
   */
  void ResetGrid(void);

  /**
   * @brief Resets the entire costmap layer including grid and observations.
   * @details Performs a complete reset of the layer by:
   * - Acquiring the voxel grid lock to ensure thread safety
   * - Resetting the costmap 2D maps
   * - Clearing the voxel grid via ResetGrid()
   * - Marking the layer as current
   * - Resetting the last updated time for all observation buffers
   * This function is called by reset() when reset is enabled.
   */
  void ResetLayer();

  // Saving grids callback for openVDB
  bool SaveGridCallback(spatio_temporal_voxel_layer::SaveGrid::Request& req,
                        spatio_temporal_voxel_layer::SaveGrid::Response& resp);

 private:
  // Sensor callbacks
  void LaserScanCallback(
      const sensor_msgs::LaserScanConstPtr& message,
      const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);
  void LaserScanValidInfCallback(
      const sensor_msgs::LaserScanConstPtr& raw_message,
      const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);
  void PointCloud2Callback(
      const sensor_msgs::PointCloud2ConstPtr& message,
      const boost::shared_ptr<buffer::MeasurementBuffer>& buffer);

  /**
   * @brief Callback for processing dynamic obstacles from obstacle_detector.
   * @details Receives obstacle clusters and buffers them as dynamic obstacle
   * readings. Extracts cluster properties including tracker ID, center,
   * radius, velocity, and extended polygons. Thread-safe operation using a
   * mutex lock.
   * @param msg Obstacle message containing detected dynamic obstacle clusters.
   */
  void ObstaclesCallback(const obstacle_detector::ObstaclesConstPtr& msg);

  /**
   * @brief Callback for processing other robot positions from RobotMotion msgs.
   * @details Receives robot motion messages and buffers them as robot motion
   * readings. Extracts robot position, velocity, radius, and footprint polygon.
   * Thread-safe operation using a mutex lock.
   * @param msg RobotMotion message containing the robot's current state.
   */
  void RobotMotionCallback(const multi_robot_public::RobotMotionsConstPtr& msg);

  // Functions for adding static obstacle zones
  bool AddStaticObservations(const observation::MeasurementReading& obs);
  bool RemoveStaticObservations(void);

  // Dynamic reconfigure
  void DynamicReconfigureCallback(dynamicReconfigureType& config,
                                  uint32_t level);

  /**
   * @brief ROS service callback to check if a point is in a blind spot.
   * @details A blind spot is defined as a location with an occupied voxel that
   * is not currently visible by any sensor frustum. The service transforms the
   * query point to the global frame and searches for occupied voxels within the
   * specified tolerance and height range.
   * @param req Service request containing the query point, tolerance, and
   *            height bounds.
   * @param resp Service response indicating success, whether a blind spot was
   *             found, and the closest occupied point if applicable.
   * @return Always returns true (ROS service convention).
   */
  bool CheckBlindSpotCallback(
      spatio_temporal_voxel_layer::CheckBlindSpot::Request& req,
      spatio_temporal_voxel_layer::CheckBlindSpot::Response& resp);

  /**
   * @brief ROS service callback to clear voxels in the robot's footprint area.
   * @details Clears all voxels within a circular region centered at the robot's
   * current position. The radius is determined by the hardware_robot_radius
   * parameter loaded from ROS parameter server. This service is useful for
   * clearing phantom obstacles that may have accumulated around the robot.
   * Thread-safe operation using a recursive mutex lock.
   * @param req Service request (empty for std_srvs::Trigger).
   * @param resp Service response containing success status and a message.
   * @return Always returns true (ROS service convention).
   */
  bool ClearRobotFootprint(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& resp);

  // Enable/Disable callback
  bool BufferEnablerCallback(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response,
      boost::shared_ptr<buffer::MeasurementBuffer>& buffer,
      boost::shared_ptr<message_filters::SubscriberBase>& subcriber);

  /**
   * @brief Publishes a visualization marker for a detected blind spot point.
   * @details Publishes a sphere marker at the blind spot location for RViz
   * visualization. If no blind spot is provided, publishes a DELETE action to
   * remove any existing marker.
   * @param world_coord The world coordinates of the blind spot, or nullopt to
   *                    clear the marker.
   */
  void PublishBlindSpotPoint(const std::optional<openvdb::Vec3d>& world_coord);

  laser_geometry::LaserProjection _laser_projector;
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> >
      _observation_subscribers;
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> >
      _observation_notifiers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> >
      _observation_buffers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> > _marking_buffers;
  std::vector<boost::shared_ptr<buffer::MeasurementBuffer> > _clearing_buffers;
  std::vector<ros::ServiceServer> _buffer_enabler_servers;
  dynamicReconfigureServerType* _dynamic_reconfigure_server;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  bool _publish_voxels, _mapping_mode;
  ros::Publisher _voxel_pub;
  ros::Duration publish_voxel_map_period_;
  ros::Time last_publish_time_;

  ros::Publisher _blind_spot_pub;
  ros::ServiceServer _grid_saver;
  ros::ServiceServer _blind_spot_checker;
  ros::ServiceServer _clear_robot_footprint_server;
  double _robot_x, _robot_y, _robot_yaw;
  double _hardware_robot_radius;

  ros::Duration _map_save_duration;
  ros::Time _last_map_save_time;
  std::string _global_frame;
  double _voxel_size, _voxel_decay;
  int _combination_method, _mark_threshold;
  volume_grid::GlobalDecayModel _decay_model;
  bool _update_footprint_enabled, _enabled, _reset_enabled;
  std::vector<geometry_msgs::Point> _transformed_footprint;
  std::vector<observation::MeasurementReading> _static_observations;
  volume_grid::SpatioTemporalVoxelGrid* _voxel_grid;
  boost::recursive_mutex _voxel_grid_lock;
  std::vector<DynamicObstacleReading> _dynamic_obstacle_readings;
  std::mutex _dynamic_obstacle_lock;
  std::vector<RobotMotionReading> _robot_motion_readings;
  std::mutex _robot_motion_lock;
};

};  // namespace spatio_temporal_voxel_layer
#endif
