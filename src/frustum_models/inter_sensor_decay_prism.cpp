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
 * Author: Seung-Hun (Hoon) Han (seunghun.han@bearrobotics.ai)
 *********************************************************************/

#include "spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp"

#include "bearlib/ros/param_loader.h"

namespace geometry {

InterSensorDecayPrism::InterSensorDecayPrism(
    PassKey, const Config& config,
    std::unique_ptr<FootprintClearingPrism> prism)
    : config_(config), prism_(std::move(prism)) {}

InterSensorDecayPrism::~InterSensorDecayPrism() = default;

std::optional<InterSensorDecayPrism::Config>
InterSensorDecayPrism::Config::Load(ros::NodeHandle& nh) {
  using namespace bear::lib::ros;

  auto prism_config = FootprintClearingPrism::Config::Load(nh);
  if (!prism_config.has_value()) {
    return std::nullopt;
  }

  Config config;
  config.prism_config = *prism_config;

  if (!config.prism_config.enable) {
    return config;
  }

  config.decay_acceleration_factor =
      LoadRequiredParam<double>(nh, "decay_acceleration_factor");

  if (!config.IsValid()) {
    return std::nullopt;
  }
  return config;
}

bool InterSensorDecayPrism::Config::IsValid() const {
  if (!prism_config.IsValid()) {
    return false;
  }
  if (prism_config.enable && decay_acceleration_factor <= 0.0) {
    ROS_ERROR(
        "InterSensorDecayPrism: decay_acceleration_factor must be positive");
    return false;
  }
  return true;
}

std::unique_ptr<InterSensorDecayPrism> InterSensorDecayPrism::Create(
    const Config& config, ros::NodeHandle nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  auto prism = FootprintClearingPrism::Create(config.prism_config, nh);
  if (!prism) {
    return nullptr;
  }
  return std::make_unique<InterSensorDecayPrism>(PassKey{}, config,
                                                 std::move(prism));
}

void InterSensorDecayPrism::SetPosition(const geometry_msgs::Point& origin) {
  prism_->SetPosition(origin);
}

void InterSensorDecayPrism::SetOrientation(
    const geometry_msgs::Quaternion& quat) {
  prism_->SetOrientation(quat);
}

void InterSensorDecayPrism::TransformModel() { prism_->TransformModel(); }

void InterSensorDecayPrism::PublishVisualization() const {
  prism_->PublishVisualization();
}

bool InterSensorDecayPrism::IsInside(const openvdb::Vec3d& point) const {
  return prism_->IsInside(point);
}

double InterSensorDecayPrism::decay_acceleration_factor() const {
  return config_.decay_acceleration_factor;
}

}  // namespace geometry
