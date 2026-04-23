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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_READING_HPP
#define SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_READING_HPP

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <cstdint>
#include <vector>

#include "obstacle_detector/ModelInfo.h"
#include "ros/ros.h"

/**
 * @struct DynamicObstacleReading
 * @brief Represents a single observation of a dynamic obstacle.
 *
 * Contains position, velocity, and geometry information for a tracked
 * dynamic obstacle at a specific point in time.
 */
struct DynamicObstacleReading {
 public:
  /// 2D point type for boost geometry operations.
  using BoostPoint = boost::geometry::model::d2::point_xy<float>;
  /// Polygon type for extended obstacle regions.
  using Polygon = boost::geometry::model::polygon<BoostPoint>;

  /// Timestamp of the obstacle observation.
  ros::Time time_;
  /// Unique identifier for this tracked obstacle.
  uint32_t tracker_id_ = 0;
  /// Position of the obstacle center in the map frame.
  Eigen::Vector2d center_ = Eigen::Vector2d::Zero();
  /// Velocity of the obstacle in m/s.
  Eigen::Vector2d velocity_ = Eigen::Vector2d::Zero();
  /// Radius of the circular bounding region in meters.
  float radius_ = 0.0f;
  /// Extended polygon regions for non-circular obstacle clearing.
  std::vector<Polygon> extended_polygons_;
  /// Model information from the obstacle detector.
  std::vector<obstacle_detector::ModelInfo> model_infos_;
};

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_READING_HPP
