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

#include <spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp>

#include "bearlib/ros/param_loader.h"
#include "ros/ros.h"
namespace geometry {

FootprintFrustum::FootprintFrustum(const Config& config, ros::NodeHandle nh)
    : nh_(nh), config_(config) {
  visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "footprint_frustum_visualization", 1);
  if (!SetFootprint(config_.footprint_points)) {
    ROS_ERROR("Failed to set footprint points.");
    ros::shutdown();
  }
}

std::optional<FootprintFrustum::Config> FootprintFrustum::Config::Load(
    ros::NodeHandle& nh) {
  using namespace bear::lib::ros;
  Config config;

  auto enable = LoadOptionalParam<bool>(nh, "enable");
  if (!enable) {
    return std::nullopt;
  }
  config.enable = *enable;

  std::vector<Point> footprint_points;
  const int kXCoord = 0;
  const int kYCoord = 1;
  int i = 0;
  while (true) {
    std::string param_name = "point" + std::to_string(i + 1);
    i++;
    auto footprint_point =
        LoadOptionalParam<std::vector<double>>(nh, param_name);
    if (!footprint_point) {
      break;
    }
    if (footprint_point->size() != 2) {
      ROS_ERROR_STREAM("FootprintFrustum: Invalid " << param_name);
      break;
    }
    ROS_DEBUG(" FootprintFrustum: %s: [%f, %f]", param_name.c_str(),
              (*footprint_point)[kXCoord], (*footprint_point)[kYCoord]);
    footprint_points.emplace_back((*footprint_point)[kXCoord],
                                  (*footprint_point)[kYCoord]);
  }
  config.footprint_points = footprint_points;
  return config;
}

FootprintFrustum::~FootprintFrustum() {}

std::unique_ptr<FootprintFrustum> FootprintFrustum::Create(const Config& config,
                                                           ros::NodeHandle nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  auto frustum =
      std::unique_ptr<FootprintFrustum>(new FootprintFrustum(config, nh));
  if (!frustum->SetFootprint(config.footprint_points)) {
    ROS_ERROR("FootprintFrustum: Failed to set footprint points.");
    return nullptr;
  }
  return frustum;
}

bool FootprintFrustum::Config::IsValid() const {
  if (footprint_points.size() < 3) {
    ROS_ERROR(
        "FootprintFrustum: At least 3 points are required to set safety "
        "zone.");
    return false;
  }
  return true;
}

bool FootprintFrustum::SetFootprint(const std::vector<Point>& points) {
  if (points.size() < 3) {
    ROS_ERROR(
        "FootprintFrustum: At least 3 points are required to set "
        "FootprintFrustum "
        "zone.");
    return false;
  }

  auto& container = original_safety_zone_polygon_.outer();
  container.clear();
  container.insert(container.end(), points.begin(), points.end());
  // Close the polygon
  boost::geometry::correct(original_safety_zone_polygon_);
  return true;
}

void FootprintFrustum::GetVisualizationMarker(
    visualization_msgs::MarkerArray& msg_list) {
  return;
}

void FootprintFrustum::TransformModel() {
  const double angle = _yaw;
  const double x0 = _position[0];
  const double y0 = _position[1];

  // clang-format off
  constexpr int kInputDimensions = 2;
  constexpr int kOutputDimensions = 2;
  boost::geometry::strategy::transform::matrix_transformer<double, kInputDimensions, kOutputDimensions>
  rotate_and_translate_back_to_robot_position(
       cos(angle), -sin(angle),  x0,
       sin(angle),  cos(angle),  y0,
       0,                   0,    1);
  // clang-format on
  boost::geometry::transform(original_safety_zone_polygon_,
                             curr_safety_zone_polygon_,
                             rotate_and_translate_back_to_robot_position);
}

bool FootprintFrustum::IsInside(const openvdb::Vec3d& pt) {
  if (!config_.enable) {
    return false;
  }
  Point boost_obstacle{pt[0], pt[1]};
  return boost::geometry::within(boost_obstacle, curr_safety_zone_polygon_);
}

void FootprintFrustum::SetPosition(const geometry_msgs::Point& origin) {
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

void FootprintFrustum::SetOrientation(const geometry_msgs::Quaternion& quat) {
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  _orientation.normalize();
  Eigen::Matrix3d rotation_matrix = _orientation.toRotationMatrix();
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  _yaw = euler_angles[0];
  _pitch = euler_angles[1];
  _roll = euler_angles[2];
}

void FootprintFrustum::PublishVisualization() {
  visualization_msgs::MarkerArray msg_list;
  visualization_msgs::Marker msg;
  for (uint i = 0; i != curr_safety_zone_polygon_.outer().size(); i++) {
    // frustum pts
    msg.header.frame_id = std::string("map");
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = 0.15;
    msg.scale.y = 0.15;
    msg.scale.z = 0.15;
    msg.pose.orientation.w = 1.0;
    msg.header.stamp = ros::Time::now();
    msg.ns = "pt_" + std::to_string(i);
    msg.color.g = 1.0f;
    msg.color.a = 1.0;
    boost::geometry::model::d2::point_xy<double> T_pt =
        curr_safety_zone_polygon_.outer()[i];
    geometry_msgs::Pose pnt;
    pnt.position.x = T_pt.x();
    pnt.position.y = T_pt.y();
    pnt.position.z = 0;  // Set Z to 0 for 2D points
    pnt.orientation.w = 1;
    msg.pose = pnt;
    msg_list.markers.push_back(msg);

    // point numbers
    msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg.ns = std::to_string(i);
    msg.pose.position.z += 0.15;
    msg.text = std::to_string(i);
    msg_list.markers.push_back(msg);
  }

  // frustum lines
  msg.header.frame_id = std::string("map");
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.scale.x = 0.15;
  msg.scale.y = 0.15;
  msg.scale.z = 0.15;
  msg.pose.orientation.w = 1.0;
  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 0;
  msg.header.stamp = ros::Time::now();
  msg.color.g = 1.0f;
  msg.color.a = 1.0;

  for (size_t i = 0; i < curr_safety_zone_polygon_.outer().size(); ++i) {
    msg.ns = "line_" + std::to_string(i);
    msg.points.clear();

    const auto& pt1 = curr_safety_zone_polygon_.outer().at(i);
    geometry_msgs::Point p1;
    p1.x = pt1.x();
    p1.y = pt1.y();
    p1.z = 0;
    msg.points.push_back(p1);

    size_t next = (i + 1) % curr_safety_zone_polygon_.outer().size();
    const auto& pt2 = curr_safety_zone_polygon_.outer().at(next);
    geometry_msgs::Point p2;
    p2.x = pt2.x();
    p2.y = pt2.y();
    p2.z = 0;
    msg.points.push_back(p2);

    msg_list.markers.push_back(msg);
  }
  visualization_pub_.publish(msg_list);
}
}  // namespace geometry
