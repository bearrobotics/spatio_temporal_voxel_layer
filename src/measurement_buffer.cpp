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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <memory>
#include <spatio_temporal_voxel_layer/measurement_buffer.hpp>

#include "spatio_temporal_voxel_layer/filter_interface.h"
namespace buffer {

/*****************************************************************************/
MeasurementBuffer::MeasurementBuffer(
    const std::string& topic_name, const double& observation_keep_time,
    const double& expected_update_rate, const double& obstacle_range,
    tf2_ros::Buffer& tf, const std::string& global_frame,
    const std::string& sensor_frame, const double& tf_tolerance,
    const double& decay_acceleration, const bool& marking, const bool& clearing,
    const double& voxel_size,
    std::unique_ptr<spatio_temporal_voxel_layer::Filter> filter,
    const bool& enabled, const bool& clear_buffer_after_reading,
    FrustumFactoryFactory::FrustumFactory frustrum_factory)
    : /*****************************************************************************/
      _buffer(tf),
      _observation_keep_time(observation_keep_time),
      _expected_update_rate(expected_update_rate),
      _last_updated(ros::Time::now()),
      _global_frame(global_frame),
      _sensor_frame(sensor_frame),
      _topic_name(topic_name),
      _obstacle_range(obstacle_range),
      _tf_tolerance(tf_tolerance),
      _decay_acceleration(decay_acceleration),
      _marking(marking),
      _clearing(clearing),
      _voxel_size(voxel_size),
      _filter(std::move(filter)),
      _enabled(enabled),
      _clear_buffer_after_reading(clear_buffer_after_reading),
      _frustrum_factory(frustrum_factory) {}
std::string MeasurementBuffer::GetTopic() const { return _topic_name; }

/*****************************************************************************/
MeasurementBuffer::~MeasurementBuffer(void)
/*****************************************************************************/
{}

/*****************************************************************************/
void MeasurementBuffer::BufferROSCloud(const sensor_msgs::PointCloud2& cloud)
/*****************************************************************************/
{
  // add a new measurement to be populated
  _observation_list.push_front(observation::MeasurementReading());

  const std::string origin_frame =
      _sensor_frame == "" ? cloud.header.frame_id : _sensor_frame;

  _observation_list.front()._sensor_name = origin_frame;

  try {
    // transform into global frame
    geometry_msgs::PoseStamped local_pose;
    local_pose.pose.position.x = 0;
    local_pose.pose.position.y = 0;
    local_pose.pose.position.z = 0;
    local_pose.pose.orientation.x = 0;
    local_pose.pose.orientation.y = 0;
    local_pose.pose.orientation.z = 0;
    local_pose.pose.orientation.w = 1;
    local_pose.header.stamp = cloud.header.stamp;
    local_pose.header.frame_id = origin_frame;

    geometry_msgs::PoseStamped global_pose;
    _buffer.canTransform(_global_frame, local_pose.header.frame_id,
                         local_pose.header.stamp, ros::Duration(0.5));
    _buffer.transform(local_pose, global_pose, _global_frame);

    _observation_list.front()._origin.x = global_pose.pose.position.x;
    _observation_list.front()._origin.y = global_pose.pose.position.y;
    _observation_list.front()._origin.z = global_pose.pose.position.z;

    _observation_list.front()._orientation = global_pose.pose.orientation;
    _observation_list.front()._obstacle_range_in_m = _obstacle_range;
    _observation_list.front()._decay_acceleration = _decay_acceleration;
    _observation_list.front()._clearing = _clearing;
    _observation_list.front()._marking = _marking;
    _observation_list.front()._frustrum_factory = _frustrum_factory;

    if (_clearing && !_marking) {
      // no need to buffer points
      return;
    }

    // transform the cloud in the global frame
    point_cloud_ptr cld_global(new sensor_msgs::PointCloud2());
    geometry_msgs::TransformStamped tf_stamped = _buffer.lookupTransform(
        _global_frame, cloud.header.frame_id, cloud.header.stamp);
    tf2::doTransform(cloud, *cld_global, tf_stamped);

    pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(*cld_global, *cloud_pcl);
    // remove points that are below or above our height restrictions, and
    // in the same time, remove NaNs and if user wants to use it, combine with a
    _filter->ApplyFilter(cloud_pcl);
    pcl_conversions::fromPCL(*cloud_pcl, *cld_global);
    _observation_list.front()._cloud = cld_global;
  } catch (tf::TransformException& ex) {
    // if fails, remove the empty observation
    _observation_list.pop_front();
    ROS_ERROR("TF Exception for sensor frame: %s, cloud frame: %s, %s",
              _sensor_frame.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  _last_updated = ros::Time::now();
  RemoveStaleObservations();
}

/*****************************************************************************/
void MeasurementBuffer::GetReadings(
    std::vector<observation::MeasurementReading>& observations)
/*****************************************************************************/
{
  RemoveStaleObservations();

  for (readings_iter it = _observation_list.begin();
       it != _observation_list.end(); ++it) {
    observations.push_back(*it);
  }
}

/*****************************************************************************/
void MeasurementBuffer::RemoveStaleObservations(void)
/*****************************************************************************/
{
  if (_observation_list.empty()) {
    return;
  }

  readings_iter it = _observation_list.begin();
  if (_observation_keep_time == ros::Duration(0.0)) {
    _observation_list.erase(++it, _observation_list.end());
    return;
  }

  for (it = _observation_list.begin(); it != _observation_list.end(); ++it) {
    observation::MeasurementReading& obs = *it;
    const ros::Duration time_diff = _last_updated - obs._cloud->header.stamp;

    if (time_diff > _observation_keep_time) {
      _observation_list.erase(it, _observation_list.end());
      return;
    }
  }
}

/*****************************************************************************/
void MeasurementBuffer::ResetAllMeasurements(void)
/*****************************************************************************/
{
  _observation_list.clear();
}

/*****************************************************************************/
bool MeasurementBuffer::ClearAfterReading(void)
/*****************************************************************************/
{
  return _clear_buffer_after_reading;
}

/*****************************************************************************/
bool MeasurementBuffer::UpdatedAtExpectedRate(void) const
/*****************************************************************************/
{
  if (_expected_update_rate == ros::Duration(0.0)) {
    return true;
  }

  const ros::Duration update_time = ros::Time::now() - _last_updated;
  const bool current = update_time.toSec() <= _expected_update_rate.toSec();
  if (!current) {
    ROS_WARN_THROTTLE(
        10., "%s buffer updated in %.2fs, it should be updated every %.2fs.",
        _topic_name.c_str(), update_time.toSec(),
        _expected_update_rate.toSec());
  }
  return current;
}

/*****************************************************************************/
bool MeasurementBuffer::IsEnabled(void) const
/*****************************************************************************/
{
  return _enabled;
}

/*****************************************************************************/
void MeasurementBuffer::SetEnabled(const bool& enabled)
/*****************************************************************************/
{
  _enabled = enabled;
}

/*****************************************************************************/
void MeasurementBuffer::ResetLastUpdatedTime(void)
/*****************************************************************************/
{
  _last_updated = ros::Time::now();
}

/*****************************************************************************/
void MeasurementBuffer::Lock(void)
/*****************************************************************************/
{
  _lock.lock();
}

/*****************************************************************************/
void MeasurementBuffer::Unlock(void)
/*****************************************************************************/
{
  _lock.unlock();
}

}  // namespace buffer
