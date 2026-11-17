/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2026, Bear Robotics, Inc.
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

#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <string>

#include "tf2/utils.h"

std::optional<RobotMotionReading> ToRobotMotionReading(
    const multi_robot_public::RobotMotion& robot_motion) {
  const double yaw = tf2::getYaw(robot_motion.orientation);
  if (!std::isfinite(robot_motion.position.x) ||
      !std::isfinite(robot_motion.position.y) || !std::isfinite(yaw) ||
      !std::isfinite(robot_motion.velocity.x) ||
      !std::isfinite(robot_motion.velocity.y) ||
      !std::isfinite(robot_motion.robot_radius)) {
    ROS_WARN_THROTTLE(1.0,
                      "[RobotMotionTracker] Dropped motion of '%s': expected "
                      "finite numbers, got x=%f y=%f yaw=%f vx=%f vy=%f "
                      "radius=%f",
                      robot_motion.robot_id.c_str(), robot_motion.position.x,
                      robot_motion.position.y, yaw, robot_motion.velocity.x,
                      robot_motion.velocity.y, robot_motion.robot_radius);
    return std::nullopt;
  }

  RobotMotionReading reading;
  reading.robot_id_ = robot_motion.robot_id;
  reading.time_ = robot_motion.time;
  reading.center_ = {robot_motion.position.x, robot_motion.position.y};
  reading.velocity_ = {robot_motion.velocity.x, robot_motion.velocity.y};
  reading.radius_ = robot_motion.robot_radius;

  const auto& footprint_points = robot_motion.robot_footprint.points;
  if (footprint_points.empty()) {
    return reading;
  }
  const Eigen::Isometry2d map_from_sender =
      Eigen::Translation2d(reading.center_) * Eigen::Rotation2Dd(yaw);
  for (const auto& point : footprint_points) {
    const Eigen::Vector2d map_point =
        map_from_sender * Eigen::Vector2d(point.x, point.y);
    reading.footprint_.outer().emplace_back(static_cast<float>(map_point.x()),
                                            static_cast<float>(map_point.y()));
  }
  boost::geometry::correct(reading.footprint_);

  std::string validity_failure;
  if (!boost::geometry::is_valid(reading.footprint_, validity_failure)) {
    ROS_WARN_THROTTLE(1.0,
                      "[RobotMotionTracker] Dropped footprint of '%s': "
                      "expected a valid polygon, got %zu points (%s)",
                      robot_motion.robot_id.c_str(), footprint_points.size(),
                      validity_failure.c_str());
    reading.footprint_.clear();
  }
  return reading;
}
