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
 * Author: Shivani Sivakumar (shivani.sivakumar@bearrobotics.ai)
 *********************************************************************/

#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"

#include <cmath>

#include "bearlib/ros/param_loader.h"
#include "tf2/utils.h"
#include "visualization_msgs/Marker.h"

namespace geometry {

FootprintClearingPrism::FootprintClearingPrism(const Config& config,
                                               ros::NodeHandle nh)
    : config_(config), nh_(nh) {
  visualization_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
}

std::optional<FootprintClearingPrism::Config>
FootprintClearingPrism::Config::Load(ros::NodeHandle& nh) {
  using bear::lib::ros::LoadOptionalParam;

  boost::optional<bool> enable = LoadOptionalParam<bool>(nh, "enable");
  if (!enable.has_value()) {
    return std::nullopt;
  }

  Config config;
  config.enable = *enable;
  config.publish_visualization =
      LoadOptionalParam<bool>(nh, "publish_visualization").value_or(true);

  if (!config.enable) {
    return config;
  }

  boost::optional<double> min_z = LoadOptionalParam<double>(nh, "min_z");
  boost::optional<double> max_z = LoadOptionalParam<double>(nh, "max_z");
  if (!min_z.has_value() || !max_z.has_value()) {
    ROS_ERROR(
        "FootprintClearingPrism: enabled prism requires both min_z and max_z");
    return std::nullopt;
  }
  config.min_z = *min_z;
  config.max_z = *max_z;

  constexpr int kXCoord = 0;
  constexpr int kYCoord = 1;
  for (int i = 1;; ++i) {
    const std::string param_name = "point" + std::to_string(i);
    boost::optional<std::vector<double>> footprint_point =
        LoadOptionalParam<std::vector<double>>(nh, param_name);
    if (!footprint_point.has_value()) {
      break;
    }
    if (footprint_point->size() != 2) {
      ROS_ERROR_STREAM("FootprintClearingPrism: Invalid " << param_name);
      return std::nullopt;
    }
    config.footprint_points.emplace_back((*footprint_point)[kXCoord],
                                         (*footprint_point)[kYCoord]);
  }

  if (!config.IsValid()) {
    return std::nullopt;
  }
  return config;
}

bool FootprintClearingPrism::Config::IsValid() const {
  if (!enable) {
    return true;
  }
  if (footprint_points.size() < 3) {
    ROS_ERROR(
        "FootprintClearingPrism: At least 3 points are required for the "
        "clearing polygon.");
    return false;
  }
  if (!(max_z > min_z)) {
    ROS_ERROR("FootprintClearingPrism: max_z must be greater than min_z.");
    return false;
  }
  return true;
}

std::unique_ptr<FootprintClearingPrism> FootprintClearingPrism::Create(
    const Config& config, ros::NodeHandle nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  std::unique_ptr<FootprintClearingPrism> prism(
      new FootprintClearingPrism(config, nh));
  if (!config.enable) {
    return prism;
  }
  if (!prism->SetFootprint(config.footprint_points)) {
    ROS_ERROR("FootprintClearingPrism: Failed to set footprint points.");
    return nullptr;
  }
  return prism;
}

FootprintClearingPrism::~FootprintClearingPrism() = default;

bool FootprintClearingPrism::SetFootprint(const std::vector<Point>& points) {
  if (points.size() < 3) {
    ROS_ERROR(
        "FootprintClearingPrism: At least 3 points are required to set the "
        "clearing polygon.");
    return false;
  }

  Polygon::ring_type& container = original_polygon_.outer();
  container.clear();
  container.insert(container.end(), points.begin(), points.end());
  boost::geometry::correct(original_polygon_);
  return true;
}

void FootprintClearingPrism::SetPosition(const geometry_msgs::Point& origin) {
  position_ = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

void FootprintClearingPrism::SetOrientation(
    const geometry_msgs::Quaternion& quat) {
  yaw_ = tf2::getYaw(quat);
}

void FootprintClearingPrism::TransformModel() {
  const double x0 = position_.x();
  const double y0 = position_.y();

  constexpr int kInputDimensions = 2;
  constexpr int kOutputDimensions = 2;
  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);
  boost::geometry::strategy::transform::matrix_transformer<
      double, kInputDimensions, kOutputDimensions>
      rotate_and_translate(cos_yaw, -sin_yaw, x0, sin_yaw, cos_yaw, y0, 0, 0,
                           1);
  boost::geometry::transform(original_polygon_, current_polygon_,
                             rotate_and_translate);
}

bool FootprintClearingPrism::IsInside(const openvdb::Vec3d& point) const {
  if (!config_.enable) {
    return false;
  }
  if (point[2] < config_.min_z || point[2] > config_.max_z) {
    return false;
  }
  return boost::geometry::within(Point(point[0], point[1]), current_polygon_);
}

void FootprintClearingPrism::PublishVisualization() const {
  if (!config_.enable || !config_.publish_visualization) {
    return;
  }

  const Polygon::ring_type& pts = current_polygon_.outer();
  if (pts.size() < 4) {
    ROS_DEBUG(
        "FootprintClearingPrism: polygon has %zu points (need at least 4), "
        "skipping visualization.",
        pts.size());
    return;
  }

  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker& delete_all = markers.markers.emplace_back();
  delete_all.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::Marker bottom;
  bottom.header.frame_id = "map";
  bottom.header.stamp = ros::Time::now();
  bottom.ns = nh_.getNamespace();
  bottom.id = 0;
  bottom.type = visualization_msgs::Marker::LINE_STRIP;
  bottom.action = visualization_msgs::Marker::ADD;
  bottom.scale.x = 0.03;
  bottom.pose.orientation.w = 1.0;
  bottom.color.r = 1.0f;
  bottom.color.g = 0.65f;
  bottom.color.a = 0.8f;

  visualization_msgs::Marker top = bottom;
  top.id = 1;

  visualization_msgs::Marker verticals = bottom;
  verticals.id = 2;
  verticals.type = visualization_msgs::Marker::LINE_LIST;

  bottom.points.reserve(pts.size());
  top.points.reserve(pts.size());
  verticals.points.reserve(pts.size() * 2);

  for (const Point& pt : pts) {
    geometry_msgs::Point bottom_pt;
    bottom_pt.x = pt.x();
    bottom_pt.y = pt.y();
    bottom_pt.z = config_.min_z;
    bottom.points.push_back(bottom_pt);

    geometry_msgs::Point top_pt = bottom_pt;
    top_pt.z = config_.max_z;
    top.points.push_back(top_pt);

    verticals.points.push_back(bottom_pt);
    verticals.points.push_back(top_pt);
  }

  markers.markers.push_back(bottom);
  markers.markers.push_back(top);
  markers.markers.push_back(verticals);
  visualization_pub_.publish(markers);
}

}  // namespace geometry
