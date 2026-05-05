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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_INTER_SENSOR_DECAY_PRISM_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_INTER_SENSOR_DECAY_PRISM_HPP_

#include <memory>
#include <optional>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "openvdb/openvdb.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"

namespace geometry {

/**
 * A robot-anchored polygonal prism that accelerates voxel decay in the
 * blind-spot region between sensors (as opposed to unconditional clearing).
 */
class InterSensorDecayPrism {
 public:
  struct PassKey {
   private:
    PassKey() = default;
    friend class InterSensorDecayPrism;
  };
  struct Config {
    static std::optional<Config> Load(ros::NodeHandle& nh);

    bool IsValid() const;

    FootprintClearingPrism::Config prism_config;
    double decay_acceleration_factor = 0.0;
  };

  static std::unique_ptr<InterSensorDecayPrism> Create(const Config& config,
                                                       ros::NodeHandle nh);

  InterSensorDecayPrism(PassKey, const Config& config,
                        std::unique_ptr<FootprintClearingPrism> prism);
  ~InterSensorDecayPrism();
  InterSensorDecayPrism(const InterSensorDecayPrism&) = delete;
  InterSensorDecayPrism& operator=(const InterSensorDecayPrism&) = delete;
  InterSensorDecayPrism(InterSensorDecayPrism&&) = delete;
  InterSensorDecayPrism& operator=(InterSensorDecayPrism&&) = delete;

  void SetPosition(const geometry_msgs::Point& origin);
  void SetOrientation(const geometry_msgs::Quaternion& quat);
  void TransformModel();
  void PublishVisualization() const;

  [[nodiscard]] bool IsInside(const openvdb::Vec3d& point) const;
  [[nodiscard]] double decay_acceleration_factor() const;

 private:
  Config config_;
  std::unique_ptr<FootprintClearingPrism> prism_;
};

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_INTER_SENSOR_DECAY_PRISM_HPP_
