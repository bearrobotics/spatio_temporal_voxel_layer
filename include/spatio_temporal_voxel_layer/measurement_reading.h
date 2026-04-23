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
 * Purpose: Measurement reading structure containing info needed for marking
 *          and frustum clearing.
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

#ifndef MEASUREMENT_READING_H_
#define MEASUREMENT_READING_H_
// msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>

#include "spatio_temporal_voxel_layer/frustum_factory.h"

namespace observation {

// Measurement Reading
struct MeasurementReading {
  /*****************************************************************************/
  MeasurementReading()
      : _cloud(new sensor_msgs::PointCloud2())
  /*****************************************************************************/
  {}

  /*****************************************************************************/
  MeasurementReading(geometry_msgs::Point& origin, std::string sensor_name,
                     sensor_msgs::PointCloud2 cloud, double obstacle_range,
                     double decay_acceleration, bool marking, bool clearing,
                     FrustumFactoryFactory::FrustumFactory frustrum_factory)
      : /*****************************************************************************/
        _origin(origin),
        _sensor_name(sensor_name),
        _cloud(new sensor_msgs::PointCloud2(cloud)),
        _obstacle_range_in_m(obstacle_range),
        _decay_acceleration(decay_acceleration),
        _marking(marking),
        _clearing(clearing),
        _frustrum_factory(frustrum_factory) {}

  /*****************************************************************************/
  MeasurementReading(sensor_msgs::PointCloud2 cloud, double obstacle_range)
      : /*****************************************************************************/
        _cloud(new sensor_msgs::PointCloud2(cloud)),
        _obstacle_range_in_m(obstacle_range) {}

  /*****************************************************************************/
  MeasurementReading(const MeasurementReading& obs)
      : /*****************************************************************************/
        _origin(obs._origin),
        _sensor_name(obs._sensor_name),
        _cloud(new sensor_msgs::PointCloud2(*(obs._cloud))),
        _obstacle_range_in_m(obs._obstacle_range_in_m),
        _marking(obs._marking),
        _clearing(obs._clearing),
        _orientation(obs._orientation),
        _decay_acceleration(obs._decay_acceleration),
        _frustrum_factory(obs._frustrum_factory) {}

  geometry_msgs::Point _origin;
  std::string _sensor_name;
  geometry_msgs::Quaternion _orientation;
  sensor_msgs::PointCloud2::Ptr _cloud;
  double _obstacle_range_in_m;
  double _marking;
  double _clearing;
  double _decay_acceleration;
  FrustumFactoryFactory::FrustumFactory _frustrum_factory;
};

}  // namespace observation

#endif  // MEASUREMENT_READING_H_
