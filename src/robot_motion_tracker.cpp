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

#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"

#include "bearlib/ros/param_loader.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"

std::optional<RobotMotionTracker::Config>
RobotMotionTracker::Config::LoadConfig(ros::NodeHandle& nh) {
  using bear::lib::ros::LoadOptionalParam;

  Config config;
  auto enable = LoadOptionalParam<bool>(nh, "enable");
  auto activation_velocity_threshold =
      LoadOptionalParam<double>(nh, "activation_velocity_threshold");
  auto min_distance_between_readings_threshold =
      LoadOptionalParam<double>(nh, "min_distance_between_readings_threshold");
  auto stale_time_threshold =
      LoadOptionalParam<double>(nh, "stale_time_threshold");
  auto inflation_radius_factor =
      LoadOptionalParam<double>(nh, "inflation_radius_factor");
  auto number_of_interpolation_circles =
      LoadOptionalParam<int>(nh, "number_of_interpolation_circles");
  auto past_time_window = LoadOptionalParam<double>(nh, "past_time_window");
  auto publish_visualization =
      LoadOptionalParam<bool>(nh, "publish_visualization");

  if (!enable || !activation_velocity_threshold ||
      !min_distance_between_readings_threshold || !stale_time_threshold ||
      !inflation_radius_factor || !number_of_interpolation_circles ||
      !past_time_window || !publish_visualization) {
    return std::nullopt;
  }

  config.enable = *enable;
  config.activation_velocity_threshold = *activation_velocity_threshold;
  config.min_distance_between_readings_threshold =
      *min_distance_between_readings_threshold;
  config.stale_time_threshold = *stale_time_threshold;
  config.inflation_radius_factor = *inflation_radius_factor;
  config.number_of_interpolation_circles = *number_of_interpolation_circles;
  config.past_time_window = *past_time_window;
  config.publish_visualization = *publish_visualization;

  return config;
}

bool RobotMotionTracker::Config::IsValid() const {
  if (activation_velocity_threshold < 0) {
    ROS_ERROR(
        "robot_motion_clearing: activation_velocity_threshold must be >= 0");
    return false;
  }
  if (min_distance_between_readings_threshold < 0) {
    ROS_ERROR(
        "robot_motion_clearing: min_distance_between_readings_threshold must "
        "be >= 0");
    return false;
  }
  if (stale_time_threshold <= 0) {
    ROS_ERROR("robot_motion_clearing: stale_time_threshold must be > 0");
    return false;
  }
  if (inflation_radius_factor <= 0) {
    ROS_ERROR("robot_motion_clearing: inflation_radius_factor must be > 0");
    return false;
  }
  if (number_of_interpolation_circles < 1) {
    ROS_ERROR(
        "robot_motion_clearing: number_of_interpolation_circles must be >= 1");
    return false;
  }
  if (past_time_window <= 0) {
    ROS_ERROR("robot_motion_clearing: past_time_window must be > 0");
    return false;
  }
  return true;
}

std::unique_ptr<RobotMotionTracker> RobotMotionTracker::Create(
    const Config& config, ros::NodeHandle& nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  return std::unique_ptr<RobotMotionTracker>(
      new RobotMotionTracker(config, nh));
}

RobotMotionTracker::RobotMotionTracker(const Config& config,
                                       ros::NodeHandle& nh)
    : config_(config), nh_(nh) {
  if (config_.publish_visualization) {
    visualization_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("visualization", 1);
  }
}

double RobotMotionTracker::SquaredDistance(const Eigen::Vector2d& a,
                                           const Eigen::Vector2d& b) {
  return (a - b).squaredNorm();
}

bool RobotMotionTracker::IsWithinMinDistanceThreshold(
    const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  const double threshold_squared =
      config_.min_distance_between_readings_threshold *
      config_.min_distance_between_readings_threshold;
  return SquaredDistance(a, b) < threshold_squared;
}

bool RobotMotionTracker::ExceedsActivationVelocity(
    const Eigen::Vector2d& velocity) const {
  return velocity.norm() > config_.activation_velocity_threshold;
}

void RobotMotionTracker::UpdateMaxRadius(float& current_max, float candidate) {
  if (candidate > current_max) {
    current_max = candidate;
  }
}

float RobotMotionTracker::LinearInterpolate(float start, float end, float t) {
  return start * (1.0f - t) + end * t;
}

void RobotMotionTracker::GenerateInterpolatedCircles(
    Circle begin_circle, Circle end_circle, int num_circles,
    std::vector<std::unique_ptr<Circle>>& interpolated_circles) const {
  for (int i = 0; i <= num_circles; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(num_circles);
    auto circle = std::make_unique<Circle>();
    circle->center.x() =
        LinearInterpolate(begin_circle.center.x(), end_circle.center.x(), t);
    circle->center.y() =
        LinearInterpolate(begin_circle.center.y(), end_circle.center.y(), t);
    circle->radius =
        LinearInterpolate(begin_circle.radius, end_circle.radius, t);
    interpolated_circles.push_back(std::move(circle));
  }
}

void RobotMotionTracker::UpdateTrackEnabledState(
    TrackInfo& track_info, const RobotMotionReading& reading) {
  if (track_info.enabled) {
    return;
  }
  if (ExceedsActivationVelocity(reading.velocity_)) {
    track_info.enabled = true;
    ROS_DEBUG_THROTTLE(
        1.0, "[RobotMotionTracker] Track '%s' enabled: velocity=%.2f m/s",
        reading.robot_id_.c_str(), reading.velocity_.norm());
  }
}

void RobotMotionTracker::UpdateExistingReading(
    TrackInfo& track_info, const RobotMotionReading& reading) {
  RobotMotionReading& last_reading = track_info.readings.back();

  if (IsWithinMinDistanceThreshold(reading.center_, last_reading.center_)) {
    last_reading.time_ = reading.time_;
    last_reading.footprint_ = reading.footprint_;
    UpdateMaxRadius(last_reading.radius_, reading.radius_);
    UpdateMaxRadius(track_info.max_radius, reading.radius_);
    UpdateTrackEnabledState(track_info, reading);
    return;
  }

  track_info.readings.push_back(reading);
  UpdateMaxRadius(track_info.max_radius, reading.radius_);
  UpdateTrackEnabledState(track_info, reading);
}

void RobotMotionTracker::AddNewReading(const RobotMotionReading& reading) {
  TrackInfo& track_info =
      track_map_.emplace(reading.robot_id_, TrackInfo()).first->second;
  track_info.readings.push_back(reading);
  track_info.max_radius = reading.radius_;
  UpdateTrackEnabledState(track_info, reading);
}

std::vector<std::unique_ptr<geometry::IClearingFrustum>>
RobotMotionTracker::GenerateRobotMotionClearingFrustums(
    std::vector<RobotMotionReading>& readings) {
  if (!config_.enable) {
    return {};
  }

  for (const auto& reading : readings) {
    auto it = track_map_.find(reading.robot_id_);
    if (it == track_map_.end()) {
      AddNewReading(reading);
      continue;
    }
    UpdateExistingReading(it->second, reading);
  }
  RemoveStaleReadings();

  std::vector<std::unique_ptr<IClearingFrustum>> frustums;
  for (auto& circle : GenerateCircles()) {
    frustums.push_back(std::move(circle));
  }
  AppendFootprintFrustums(readings, frustums);

  if (config_.publish_visualization) {
    PublishVisualization();
  }

  return frustums;
}

void RobotMotionTracker::AppendFootprintFrustums(
    const std::vector<RobotMotionReading>& readings,
    std::vector<std::unique_ptr<IClearingFrustum>>& frustums) const {
  for (const RobotMotionReading& reading : readings) {
    if (reading.footprint_.outer().empty()) {
      continue;
    }
    auto polygon_frustum = std::make_unique<geometry::Polygon>();
    // RobotMotionReading::Polygon and DynamicObstacleReading::Polygon are the
    // same underlying boost geometry type, so this assignment is valid.
    polygon_frustum->polygon_ = reading.footprint_;
    frustums.push_back(std::move(polygon_frustum));
  }
}

std::vector<std::unique_ptr<RobotMotionTracker::Circle>>
RobotMotionTracker::GenerateCircles() {
  std::vector<std::unique_ptr<Circle>> circles;
  const ros::Time cutoff_time =
      ros::Time::now() - ros::Duration(config_.past_time_window);

  for (const auto& [robot_id, track_info] : track_map_) {
    if (!track_info.enabled) {
      continue;
    }

    Circle previous_circle;
    bool has_previous_point = false;

    for (const RobotMotionReading& reading : track_info.readings) {
      if (reading.time_ < cutoff_time) {
        continue;
      }

      Circle current_circle;
      current_circle.center = reading.center_;
      current_circle.radius =
          track_info.max_radius * config_.inflation_radius_factor;

      if (has_previous_point) {
        GenerateInterpolatedCircles(previous_circle, current_circle,
                                    config_.number_of_interpolation_circles,
                                    circles);
      }

      previous_circle = current_circle;
      has_previous_point = true;
    }
  }
  return circles;
}

void RobotMotionTracker::RemoveStaleReadings() {
  for (auto it = track_map_.begin(); it != track_map_.end();) {
    const double time_diff =
        (ros::Time::now() - it->second.readings.back().time_).toSec();
    if (time_diff > config_.stale_time_threshold) {
      it = track_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std_msgs::ColorRGBA RobotMotionTracker::GetTrackColor(bool enabled) {
  std_msgs::ColorRGBA color;
  color.a = 0.25f;
  color.r = 1.0f;
  color.g = 64.7f;
  color.b = 0.f;
  return color;
}

void RobotMotionTracker::AppendCircleMarkers(
    const TrackInfo& track_info, const std::string& robot_id,
    const ros::Time& now, const ros::Time& cutoff_time,
    const std_msgs::ColorRGBA& color, int& marker_id,
    visualization_msgs::MarkerArray& marker_array) {
  const std::string marker_namespace = "robot_track_" + robot_id;
  const float inflated_radius =
      track_info.max_radius * config_.inflation_radius_factor;
  constexpr float kCylinderHeight = 0.01f;

  Circle previous_circle;
  bool has_previous_reading = false;

  for (const RobotMotionReading& reading : track_info.readings) {
    if (reading.time_ < cutoff_time) {
      continue;
    }

    Circle current_circle;
    current_circle.center = reading.center_;
    current_circle.radius = inflated_radius;

    if (!has_previous_reading) {
      previous_circle = current_circle;
      has_previous_reading = true;
      continue;
    }

    std::vector<std::unique_ptr<Circle>> interpolated_circles;
    GenerateInterpolatedCircles(previous_circle, current_circle,
                                config_.number_of_interpolation_circles,
                                interpolated_circles);

    for (const auto& circle : interpolated_circles) {
      visualization_msgs::Marker& marker = marker_array.markers.emplace_back();
      marker.header.frame_id = "map";
      marker.header.stamp = now;
      marker.ns = marker_namespace;
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = circle->center.x();
      marker.pose.position.y = circle->center.y();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = circle->radius * 2.0;
      marker.scale.y = circle->radius * 2.0;
      marker.scale.z = kCylinderHeight;
      marker.color = color;
    }

    previous_circle = current_circle;
  }
}

void RobotMotionTracker::AppendFootprintMarkers(
    const TrackInfo& track_info, const std::string& robot_id,
    const ros::Time& now, const std_msgs::ColorRGBA& color, int& marker_id,
    visualization_msgs::MarkerArray& marker_array) {
  const std::string marker_namespace = "robot_footprint_" + robot_id;
  constexpr float kLineWidth = 0.02f;

  for (const RobotMotionReading& reading : track_info.readings) {
    if (reading.footprint_.outer().empty()) {
      continue;
    }
    visualization_msgs::Marker& marker = marker_array.markers.emplace_back();
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = marker_namespace;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = kLineWidth;
    marker.color = color;

    const auto& outer_ring = reading.footprint_.outer();
    for (const auto& point : outer_ring) {
      geometry_msgs::Point& p = marker.points.emplace_back();
      p.x = boost::geometry::get<0>(point);
      p.y = boost::geometry::get<1>(point);
      p.z = 0.0;
    }

    if (outer_ring.empty()) {
      continue;
    }

    geometry_msgs::Point& closing_point = marker.points.emplace_back();
    closing_point.x = boost::geometry::get<0>(outer_ring.front());
    closing_point.y = boost::geometry::get<1>(outer_ring.front());
    closing_point.z = 0.0;
  }
}

void RobotMotionTracker::PublishVisualization() {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker& delete_all_marker =
      marker_array.markers.emplace_back();
  delete_all_marker.action = visualization_msgs::Marker::DELETEALL;

  const ros::Time now = ros::Time::now();
  const ros::Time cutoff_time = now - ros::Duration(config_.past_time_window);
  int marker_id = 0;

  for (const auto& [robot_id, track_info] : track_map_) {
    if (!track_info.enabled) {
      continue;
    }
    const std_msgs::ColorRGBA color = GetTrackColor(track_info.enabled);
    AppendCircleMarkers(track_info, robot_id, now, cutoff_time, color,
                        marker_id, marker_array);
    AppendFootprintMarkers(track_info, robot_id, now, color, marker_id,
                           marker_array);
  }

  visualization_pub_.publish(marker_array);
}
