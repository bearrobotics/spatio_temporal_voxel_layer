#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_READING_HPP
#define SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_READING_HPP

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <string>

#include "ros/ros.h"

/**
 * @struct RobotMotionReading
 * @brief Represents a single observation of another robot from a RobotMotion
 *        message.
 *
 * Contains position, velocity, radius, and footprint polygon for a robot
 * tracked by robot_id.
 */
struct RobotMotionReading {
  /// 2D point type for boost geometry operations.
  using BoostPoint = boost::geometry::model::d2::point_xy<float>;
  /// Polygon type for the robot footprint region.
  using Polygon = boost::geometry::model::polygon<BoostPoint>;

  /// Timestamp of the robot motion observation.
  ros::Time time_;
  /// Unique string identifier for the robot.
  std::string robot_id_;
  /// Position of the robot center in the map frame.
  Eigen::Vector2d center_ = Eigen::Vector2d::Zero();
  /// Velocity of the robot in m/s.
  Eigen::Vector2d velocity_ = Eigen::Vector2d::Zero();
  /// Radius of the robot (robot_radius field) in meters.
  float radius_ = 0.0f;
  /// Robot footprint polygon in the map frame.
  Polygon footprint_;
};

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_READING_HPP
