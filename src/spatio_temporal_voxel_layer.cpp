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
 *  - Inter-sensor decay prism that accelerates voxel decay in the
 *    blind-spot region between sensors
 *  - CheckBlindSpot and ClearRobotFootprint services
 *  - Sensor data filtering (noise filter, frustum-based filtering)
 *  - Safety zone frustum support
 *  - Various bug fixes and performance improvements
 *    (see git history for detailed per-commit changes)
 * Contributors:
 *  - Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *  - Shivani Sivakumar (shivani.sivakumar@bearrobotics.ai)
 *  - Hashir Zahir (hashir.zahir@bearrobotics.ai)
 *  - Seung-Hun (Hoon) Han (seunghun.han@bearrobotics.ai)
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

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

#include "bearlib/ros/param_loader.h"
#include "spatio_temporal_voxel_layer/filter_factory.h"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace spatio_temporal_voxel_layer {

/*****************************************************************************/
SpatioTemporalVoxelLayer::SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{}

/*****************************************************************************/
SpatioTemporalVoxelLayer::~SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
  if (_dynamic_reconfigure_server) {
    delete _dynamic_reconfigure_server;
  }
  if (_voxel_grid) {
    delete _voxel_grid;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::onInitialize(void)
/*****************************************************************************/
{
  ROS_INFO("%s being initialized as SpatioTemporalVoxelLayer!",
           getName().c_str());

  // initialize parameters, grid, and sub/pubs
#ifdef BEAR_COSTMAP_PARENT_NH
  ros::NodeHandle nh(parent_nh_, name_), g_nh, prefix_nh;
#else
  ros::NodeHandle nh("~/" + name_), g_nh, prefix_nh;
#endif

  _global_frame = std::string(layered_costmap_->getGlobalFrameID());

  // source names
  std::string topics_string =
      bear::lib::ros::LoadRequiredParam<std::string>(nh, "observation_sources");
  // timeout in seconds for transforms
  double transform_tolerance =
      bear::lib::ros::LoadRequiredParam<double>(nh, "transform_tolerance");
  // whether to default on
  _enabled = bear::lib::ros::LoadRequiredParam<bool>(nh, "enabled");
  enabled_ = _enabled;  // costmap_2d for some unexplicable reason uses globals
  // publish the voxel grid to visualize
  _publish_voxels =
      bear::lib::ros::LoadRequiredParam<bool>(nh, "publish_voxel_map");
  publish_voxel_map_period_ =
      ros::Duration(bear::lib::ros::LoadRequiredParam<double>(
          nh, "publish_voxel_map_period"));

  // size of each voxel in meters
  _voxel_size = layered_costmap_->getCostmap()->getResolution() *
                bear::lib::ros::LoadRequiredParam<double>(
                    nh, "voxel_size_costmap_resolution_multiplier");
  // 1=takes highest in layers, 0=takes current layer
  _combination_method =
      bear::lib::ros::LoadRequiredParam<int>(nh, "combination_method");
  // number of voxels per vertical needed to have obstacle
  _mark_threshold =
      bear::lib::ros::LoadRequiredParam<int>(nh, "mark_threshold");
  // clear under robot footprint
  _update_footprint_enabled =
      bear::lib::ros::LoadRequiredParam<bool>(nh, "update_footprint_enabled");
  // whether reset() function of the costmap interface is enabled. This is
  // helpful for now to remember obstacles in between destinations. Eventually
  // we need to make this smarter.
  _reset_enabled = bear::lib::ros::LoadRequiredParam<bool>(nh, "reset_enabled");

  // keep tabs on unknown space
  bool track_unknown_space =
      bear::lib::ros::LoadOptionalParam<bool>(nh, "track_unknown_space")
          .value_or(layered_costmap_->isTrackingUnknown());
  int decay_model_int =
      bear::lib::ros::LoadRequiredParam<int>(nh, "decay_model");
  _decay_model = static_cast<volume_grid::GlobalDecayModel>(decay_model_int);
  // decay param
  _voxel_decay = bear::lib::ros::LoadRequiredParam<double>(nh, "voxel_decay");

  // Load hardware robot radius for footprint clearing
  _hardware_robot_radius =
      bear::lib::ros::LoadRequiredParam<double>(nh, "hardware_robot_radius");

  // whether to map or navigate
  _mapping_mode = bear::lib::ros::LoadRequiredParam<bool>(nh, "mapping_mode");

  // if mapping, how often to save a map for safety
  double map_save_time =
      bear::lib::ros::LoadOptionalParam<double>(nh, "map_save_duration")
          .value_or(60.0);

  ros::NodeHandle safety_zone_nh(nh, "safety_zone");
  auto safety_zone_config =
      geometry::FootprintFrustum::Config::Load(safety_zone_nh);
  if (!safety_zone_config) {
    ROS_FATAL("Failed to load safety zone footprint frustum configuration.");
    std::terminate();
  }
  auto safety_zone_frustum =
      geometry::FootprintFrustum::Create(*safety_zone_config, safety_zone_nh);
  if (!safety_zone_frustum) {
    ROS_FATAL("Failed to create safety zone footprint frustum.");
    std::terminate();
  }

  std::unique_ptr<geometry::InterSensorDecayPrism> inter_sensor_decay_prism;
  ros::NodeHandle inter_sensor_decay_prism_nh(nh, "inter_sensor_decay_prism");
  if (inter_sensor_decay_prism_nh.hasParam("enable")) {
    auto inter_sensor_decay_prism_config =
        geometry::InterSensorDecayPrism::Config::Load(
            inter_sensor_decay_prism_nh);
    if (!inter_sensor_decay_prism_config) {
      ROS_FATAL("Failed to load inter-sensor decay prism config.");
      std::terminate();
    }
    if (inter_sensor_decay_prism_config->prism_config.enable) {
      inter_sensor_decay_prism = geometry::InterSensorDecayPrism::Create(
          *inter_sensor_decay_prism_config, inter_sensor_decay_prism_nh);
      if (!inter_sensor_decay_prism) {
        ROS_FATAL("Failed to create inter-sensor decay prism.");
        std::terminate();
      }
    }
  }

  std::unique_ptr<geometry::FootprintClearingPrism>
      front_blind_spot_clearing_prism;
  ros::NodeHandle front_blind_spot_clearing_prism_nh(
      nh, "front_blind_spot_clearing_prism");
  if (front_blind_spot_clearing_prism_nh.hasParam("enable")) {
    std::optional<geometry::FootprintClearingPrism::Config>
        front_blind_spot_clearing_prism_config =
            geometry::FootprintClearingPrism::Config::Load(
                front_blind_spot_clearing_prism_nh);
    if (!front_blind_spot_clearing_prism_config) {
      ROS_FATAL("Failed to load front blind-spot clearing prism config.");
      std::terminate();
    }
    if (front_blind_spot_clearing_prism_config->enable) {
      front_blind_spot_clearing_prism =
          geometry::FootprintClearingPrism::Create(
              *front_blind_spot_clearing_prism_config,
              front_blind_spot_clearing_prism_nh);
      if (!front_blind_spot_clearing_prism) {
        ROS_FATAL("Failed to create front blind-spot clearing prism.");
        std::terminate();
      }
    }
  }

  ros::NodeHandle dynamic_obstacle_clearing_nh(nh, "dynamic_obstacle_clearing");
  auto config =
      DynamicObstacleTracker::Config::LoadConfig(dynamic_obstacle_clearing_nh);
  if (!config) {
    ROS_FATAL("Failed to load dynamic obstacle tracker configuration.");
    std::terminate();
  }
  auto dynamic_obstacle_tracker =
      DynamicObstacleTracker::Create(*config, dynamic_obstacle_clearing_nh);
  if (!dynamic_obstacle_tracker) {
    ROS_FATAL("Failed to create dynamic obstacle tracker.");
    std::terminate();
  }

  ros::NodeHandle robot_motion_clearing_nh(nh, "robot_motion_clearing");
  auto rm_config =
      RobotMotionTracker::Config::LoadConfig(robot_motion_clearing_nh);
  if (!rm_config) {
    ROS_FATAL("Failed to load robot motion tracker configuration.");
    std::terminate();
  }
  auto robot_motion_tracker =
      RobotMotionTracker::Create(*rm_config, robot_motion_clearing_nh);
  if (!robot_motion_tracker) {
    ROS_FATAL("Failed to create robot motion tracker.");
    std::terminate();
  }

  if (_mapping_mode) {
    _map_save_duration = ros::Duration(map_save_time);
    _last_map_save_time = ros::Time::now() - _map_save_duration;
  }

  if (track_unknown_space) {
    setDefaultValue(costmap_2d::NO_INFORMATION);
  } else {
    setDefaultValue(costmap_2d::FREE_SPACE);
  }

  _voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
  _blind_spot_pub =
      nh.advertise<visualization_msgs::Marker>("blind_spot_point", 1);
  _grid_saver =
      nh.advertiseService("spatiotemporal_voxel_grid/save_grid",
                          &SpatioTemporalVoxelLayer::SaveGridCallback, this);

  _voxel_grid = new volume_grid::SpatioTemporalVoxelGrid(
      _voxel_size, (double)getDefaultValue(), _decay_model, _voxel_decay,
      _publish_voxels, std::move(safety_zone_frustum),
      std::move(inter_sensor_decay_prism),
      std::move(front_blind_spot_clearing_prism),
      std::move(dynamic_obstacle_tracker), std::move(robot_motion_tracker));
  matchSize();
  current_ = true;
  _blind_spot_checker = nh.advertiseService(
      "check_blind_spot", &SpatioTemporalVoxelLayer::CheckBlindSpotCallback,
      this);

  _clear_robot_footprint_server =
      nh.advertiseService("clear_robot_footprint",
                          &SpatioTemporalVoxelLayer::ClearRobotFootprint, this);

  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
  std::stringstream ss(topics_string);
  std::string source;

  std::vector<FrustumFactoryFactory::FrustumFactory> frustum_factories;
  while (ss >> source) {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate;
    double decay_acceleration;
    std::string topic, sensor_frame, data_type, filter_str;
    bool inf_is_valid, clearing, marking, clear_after_reading, enabled;

    topic =
        bear::lib::ros::LoadRequiredParam<std::string>(source_node, "topic");
    sensor_frame = bear::lib::ros::LoadRequiredParam<std::string>(
        source_node, "sensor_frame");
    observation_keep_time = bear::lib::ros::LoadRequiredParam<double>(
        source_node, "observation_persistence");
    expected_update_rate = bear::lib::ros::LoadRequiredParam<double>(
        source_node, "expected_update_rate");
    data_type = bear::lib::ros::LoadRequiredParam<std::string>(source_node,
                                                               "data_type");
    inf_is_valid =
        bear::lib::ros::LoadRequiredParam<bool>(source_node, "inf_is_valid");

    clearing = bear::lib::ros::LoadRequiredParam<bool>(source_node, "clearing");
    marking = bear::lib::ros::LoadRequiredParam<bool>(source_node, "marking");

    ros::NodeHandle frustrum_nh(source_node, "frustrum");
    auto frustrum_factory =
        FrustumFactoryFactory::CreateFrustumFactory(frustrum_nh);
    if (!frustrum_factory) {
      ROS_FATAL("Failed to create frustum factory for source: %s",
                source.c_str());
      std::terminate();
    }
    frustum_factories.push_back(frustrum_factory);

    // acceleration scales the model's decay in presence of readings
    decay_acceleration = bear::lib::ros::LoadRequiredParam<double>(
        source_node, "decay_acceleration");

    // clears measurement buffer after reading values from it
    clear_after_reading = bear::lib::ros::LoadRequiredParam<bool>(
        source_node, "clear_after_reading");
    // Whether the frustum is enabled on startup. Can be toggled with service
    enabled = bear::lib::ros::LoadRequiredParam<bool>(source_node, "enabled");

    // Apply a PCL filter (Approximate VoxeGrid or PassThrough) or skip
    ros::NodeHandle filter_nh(source_node, "filter");
    std::unique_ptr<Filter> filter = FilterFactory::CreateFilter(filter_nh);
    if (!filter) {
      ROS_FATAL("Failed to create filter of type: %s", filter_str.c_str());
      std::terminate();
    }

    if (!sensor_frame.empty()) {
      sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      throw std::runtime_error(
          "Only topics that use pointclouds or laser scans are supported.");
    }

    std::string obstacle_range_param_name;
    double obstacle_range = 3.0;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name)) {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // create an observation buffer
    _observation_buffers.push_back(boost::shared_ptr<buffer::MeasurementBuffer>(
        new buffer::MeasurementBuffer(
            topic, observation_keep_time, expected_update_rate, obstacle_range,
            tf_buffer_, _global_frame, sensor_frame, transform_tolerance,
            decay_acceleration, marking, clearing, _voxel_size,
            std::move(filter), enabled, clear_after_reading,
            frustrum_factory)));

    // Add buffer to marking observation buffers
    if (marking == true) {
      _marking_buffers.push_back(_observation_buffers.back());
    }

    // Add buffer to clearing observation buffers
    if (clearing == true) {
      _clearing_buffers.push_back(_observation_buffers.back());
    }

    // create a callback for the topic
    if (data_type == "LaserScan") {
      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
          sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(
              g_nh, topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(
              *sub, tf_buffer_, _global_frame, 50, 0));

      if (inf_is_valid) {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanValidInfCallback,
                        this, _1, _observation_buffers.back()));
      } else {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanCallback, this, _1,
                        _observation_buffers.back()));
      }

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);

      _observation_notifiers.back()->setTolerance(ros::Duration(0.05));
    }

    else if (data_type == "PointCloud2") {
      auto sub = boost::make_shared<
          message_filters::Subscriber<sensor_msgs::PointCloud2>>(g_nh, topic,
                                                                 50);
      _observation_subscribers.push_back(sub);

      auto filter =
          boost::make_shared<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>>(
              *sub, tf_buffer_, _global_frame, 50, g_nh);

      filter->registerCallback(
          boost::bind(&SpatioTemporalVoxelLayer::PointCloud2Callback, this, _1,
                      _observation_buffers.back()));

      _observation_notifiers.push_back(filter);
    }

    ros::ServiceServer server;
    boost::function<bool(std_srvs::SetBool::Request&,
                         std_srvs::SetBool::Response&)>
        serv_callback;

    serv_callback = boost::bind(
        &SpatioTemporalVoxelLayer::BufferEnablerCallback, this, _1, _2,
        _observation_buffers.back(), _observation_subscribers.back());

    std::string toggle_topic = source + "/toggle_enabled";
    server = nh.advertiseService(toggle_topic, serv_callback);

    _buffer_enabler_servers.push_back(server);

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.reserve(2);
      target_frames.push_back(_global_frame);
      target_frames.push_back(sensor_frame);
      _observation_notifiers.back()->setTargetFrames(target_frames);
    }
  }

  // Setup dynamic obstacle tracker
  auto sub = boost::make_shared<
      message_filters::Subscriber<obstacle_detector::Obstacles>>(
      g_nh, "/obstacle_detector/obstacles", 50);
  _observation_subscribers.push_back(sub);
  auto filter =
      boost::make_shared<tf2_ros::MessageFilter<obstacle_detector::Obstacles>>(
          *sub, tf_buffer_, _global_frame, 50, g_nh);
  filter->registerCallback(
      boost::bind(&SpatioTemporalVoxelLayer::ObstaclesCallback, this, _1));
  _observation_notifiers.push_back(filter);

  // Setup robot motion tracker — positions are already in the map frame so no
  // TF filter is needed.
  std::string robot_motion_topic =
      bear::lib::ros::LoadRequiredParam<std::string>(robot_motion_clearing_nh,
                                                     "topic");
  auto rm_sub = boost::make_shared<
      message_filters::Subscriber<multi_robot_public::RobotMotions>>(
      g_nh, robot_motion_topic, 50);
  rm_sub->registerCallback(
      boost::bind(&SpatioTemporalVoxelLayer::RobotMotionCallback, this, _1));
  _observation_subscribers.push_back(rm_sub);

  // Dynamic reconfigure
  // TODO: @benenati RN-1794 Update dynamic reconfigure to work with new changes
  // dynamic_reconfigure::Server<dynamicReconfigureType>::CallbackType f;
  // f = boost::bind(&SpatioTemporalVoxelLayer::DynamicReconfigureCallback,
  // this,
  //                 _1, _2);
  // _dynamic_reconfigure_server = new dynamicReconfigureServerType(nh);
  // _dynamic_reconfigure_server->setCallback(f);

  ROS_INFO("%s initialization complete!", getName().c_str());
}

bool SpatioTemporalVoxelLayer::ClearRobotFootprint(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  _voxel_grid->ClearCircularArea(_robot_x, _robot_y, _hardware_robot_radius);

  resp.success = true;
  resp.message = "Cleared robot footprint";
  return true;
}

void SpatioTemporalVoxelLayer::ObstaclesCallback(
    const obstacle_detector::ObstaclesConstPtr& msg) {
  // Technically we need to convert the locations to the correct frame but
  // they're already in "map"
  std::scoped_lock lock(_dynamic_obstacle_lock);
  for (const auto& cluster : msg->clusters) {
    DynamicObstacleReading& reading = _dynamic_obstacle_readings.emplace_back();
    reading.tracker_id_ = cluster.tracker_id;
    reading.center_[0] = cluster.center.x;
    reading.center_[1] = cluster.center.y;
    reading.radius_ = cluster.radius;
    reading.velocity_[0] = cluster.velocity.x;
    reading.velocity_[1] = cluster.velocity.y;
    reading.time_ = cluster.header.stamp;
    reading.model_infos_ = cluster.model_infos;

    // Generate extended polygons for the dynamic obstacle
    for (const auto& polygon : cluster.extended_polygons) {
      DynamicObstacleReading::Polygon& extended_polygon =
          reading.extended_polygons_.emplace_back();
      for (const geometry_msgs::Point32& point : polygon.points) {
        extended_polygon.outer().emplace_back(point.x, point.y);
      }
    }
  }
}

bool SpatioTemporalVoxelLayer::GetDynamicObstacleReadings(
    std::vector<DynamicObstacleReading>& dynamic_obstacle_readings)
/*****************************************************************************/
{
  // get dynamic obstacle readings
  bool current = true;
  ROS_WARN_THROTTLE(10, "Getting dynamic obstacle readings: Size: %i",
                    _dynamic_obstacle_readings.size());

  std::scoped_lock lock(_dynamic_obstacle_lock);
  dynamic_obstacle_readings = _dynamic_obstacle_readings;
  _dynamic_obstacle_readings.clear();
  return current;
}

void SpatioTemporalVoxelLayer::RobotMotionCallback(
    const multi_robot_public::RobotMotionsConstPtr& msg) {
  std::scoped_lock lock(_robot_motion_lock);
  for (const auto& robot : msg->robot_list) {
    RobotMotionReading& reading = _robot_motion_readings.emplace_back();
    reading.robot_id_ = robot.robot_id;
    reading.time_ = robot.time;
    reading.center_ = {robot.position.x, robot.position.y};
    reading.velocity_ = {robot.velocity.x, robot.velocity.y};
    reading.radius_ = robot.robot_radius;
    for (const auto& pt : robot.robot_footprint.points) {
      reading.footprint_.outer().emplace_back(pt.x, pt.y);
    }
  }
}

bool SpatioTemporalVoxelLayer::GetRobotMotionReadings(
    std::vector<RobotMotionReading>& robot_motion_readings) {
  std::scoped_lock lock(_robot_motion_lock);
  robot_motion_readings = _robot_motion_readings;
  _robot_motion_readings.clear();
  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanCallback(
    const sensor_msgs::LaserScanConstPtr& message,
    const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // laser scan where infinity is invalid callback function
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;
  try {
    _laser_projector.transformLaserScanToPointCloud(
        message->header.frame_id, *message, cloud, tf_buffer_);
  } catch (tf::TransformException& ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s",
             _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(*message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanValidInfCallback(
    const sensor_msgs::LaserScanConstPtr& raw_message,
    const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // Filter infinity to max_range
  float epsilon = 0.0001;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    _laser_projector.transformLaserScanToPointCloud(message.header.frame_id,
                                                    message, cloud, tf_buffer_);
  } catch (tf::TransformException& ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s",
             _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::PointCloud2Callback(
    const sensor_msgs::PointCloud2ConstPtr& message,
    const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::BufferEnablerCallback(
    std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response,
    boost::shared_ptr<buffer::MeasurementBuffer>& buffer,
    boost::shared_ptr<message_filters::SubscriberBase>& subcriber)
/*****************************************************************************/
{
  buffer->Lock();
  if (buffer->IsEnabled() != request.data) {
    buffer->SetEnabled(request.data);
    if (request.data) {
      subcriber->subscribe();
      buffer->ResetLastUpdatedTime();
      response.message = "Enabling sensor";
    } else if (subcriber) {
      subcriber->unsubscribe();
      response.message = "Disabling sensor";
    }
  } else {
    response.message = "Sensor already in the required state doing nothing";
  }
  buffer->Unlock();
  response.success = true;
  return response.success;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetMarkingObservations(
    std::vector<observation::MeasurementReading>& marking_observations) const
/*****************************************************************************/
{
  // get marking observations and static marked areas
  bool current = true;

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i) {
    _marking_buffers[i]->Lock();
    _marking_buffers[i]->GetReadings(marking_observations);
    current = _marking_buffers[i]->UpdatedAtExpectedRate();
    _marking_buffers[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              _static_observations.begin(),
                              _static_observations.end());
  return current;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetClearingObservations(
    std::vector<observation::MeasurementReading>& clearing_observations) const
/*****************************************************************************/
{
  // get clearing observations
  bool current = true;
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i) {
    _clearing_buffers[i]->Lock();
    _clearing_buffers[i]->GetReadings(clearing_observations);
    current = _clearing_buffers[i]->UpdatedAtExpectedRate();
    _clearing_buffers[i]->Unlock();
  }
  return current;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ObservationsResetAfterReading() const
/*****************************************************************************/
{
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i) {
    _clearing_buffers[i]->Lock();
    if (_clearing_buffers[i]->ClearAfterReading()) {
      _clearing_buffers[i]->ResetAllMeasurements();
    }
    _clearing_buffers[i]->Unlock();
  }

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i) {
    _marking_buffers[i]->Lock();
    if (_marking_buffers[i]->ClearAfterReading()) {
      _marking_buffers[i]->ResetAllMeasurements();
    }
    _marking_buffers[i]->Unlock();
  }
  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::updateFootprint(double robot_x, double robot_y,
                                               double robot_yaw, double* min_x,
                                               double* min_y, double* max_x,
                                               double* max_y)
/*****************************************************************************/
{
  _robot_x = robot_x;
  _robot_y = robot_y;
  _robot_yaw = robot_yaw;

  // updates layer costmap to include footprint for clearing in voxel grid
  if (!_update_footprint_enabled) {
    return false;
  }
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(),
                                 _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++) {
    touch(_transformed_footprint[i].x, _transformed_footprint[i].y, min_x,
          min_y, max_x, max_y);
  }
  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  ROS_INFO("%s was activated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it) {
    (*sub_it)->subscribe();
  }
  observation_buffers_iter buf_it = _observation_buffers.begin();
  for (buf_it; buf_it != _observation_buffers.end(); ++buf_it) {
    (*buf_it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  ROS_INFO("%s was deactivated.", getName().c_str());
  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it) {
    if (*sub_it != NULL) {
      (*sub_it)->unsubscribe();
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::reset(void)
/*****************************************************************************/
{
  if (!_reset_enabled) {
    return;
  }
  ResetLayer();
}

void SpatioTemporalVoxelLayer::ResetLayer() {
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  // reset layer
  Costmap2D::resetMaps();
  this->ResetGrid();
  current_ = true;
  observation_buffers_iter it = _observation_buffers.begin();
  for (it; it != _observation_buffers.end(); ++it) {
    (*it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::AddStaticObservations(
    const observation::MeasurementReading& obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not marked
  ROS_INFO("%s: Adding static observation to map.", getName().c_str());

  try {
    _static_observations.push_back(obs);
    return true;
  } catch (...) {
    ROS_WARN("Could not add static observations to voxel layer");
    return false;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::RemoveStaticObservations(void)
/*****************************************************************************/
{
  // kill all static observations added to each update cycle
  ROS_INFO("%s: Removing static observations to map.", getName().c_str());

  try {
    _static_observations.clear();
    return true;
  } catch (...) {
    ROS_WARN("Couldn't remove static observations from %s.", getName().c_str());
    return false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::DynamicReconfigureCallback(
    SpatioTemporalVoxelLayerConfig& config, uint32_t level)
/*****************************************************************************/
{
  // TODO: @benenati Update dynamic reconfigure to work with new changes.
  // Currently
  //  this is disabled to be able to ship MVP.
  //  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  // _enabled = config.enabled;
  // _combination_method = config.combination_method;
  // _mark_threshold = config.mark_threshold;
  // _update_footprint_enabled = config.update_footprint_enabled;
  // _mapping_mode = config.mapping_mode;
  // _map_save_duration = ros::Duration(config.map_save_duration);

  // if (level >= 1)  // update grid
  // {
  //   auto default_value = (config.track_unknown_space)
  //                            ? costmap_2d::NO_INFORMATION
  //                            : costmap_2d::FREE_SPACE;
  //   setDefaultValue(default_value);
  //   _voxel_size = config.voxel_size_costmap_resolution_multiplier *
  //                 layered_costmap_->getCostmap()->getResolution();
  //   _voxel_decay = config.voxel_decay;
  //   _decay_model =
  //       static_cast<volume_grid::GlobalDecayModel>(config.decay_model);
  //   _publish_voxels = config.publish_voxel_map;

  //   delete _voxel_grid;
  //   _voxel_grid = new volume_grid::SpatioTemporalVoxelGrid(
  //       _voxel_size, static_cast<double>(getDefaultValue()), _decay_model,
  //       _voxel_decay, _publish_voxels, nullptr, nullptr);
  // }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ResetGrid(void)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid()) {
    ROS_WARN("Did not clear level set in %s!", getName().c_str());
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                           int min_i, int min_j, int max_i,
                                           int max_j)
/*****************************************************************************/
{
  // update costs in master_grid with costmap_
  if (!_enabled) {
    return;
  }

  if (_update_footprint_enabled) {
    setConvexPolygonCost(_transformed_footprint, costmap_2d::FREE_SPACE);
  }

  switch (_combination_method) {
    case 0:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    case 1:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    default:
      break;
  }
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::UpdateROSCostmap(
    double* min_x, double* min_y, double* max_x, double* max_y,
    std::unordered_set<volume_grid::occupany_cell>& cleared_cells)
/*****************************************************************************/
{
  // grabs map of occupied cells from grid and adds to costmap_
  Costmap2D::resetMaps();

  std::unordered_map<volume_grid::occupany_cell, uint>::iterator it;
  for (it = _voxel_grid->GetFlattenedCostmap()->begin();
       it != _voxel_grid->GetFlattenedCostmap()->end(); ++it) {
    uint map_x, map_y;
    if (it->second >= _mark_threshold &&
        worldToMap(it->first.x, it->first.y, map_x, map_y)) {
      costmap_[getIndex(map_x, map_y)] = costmap_2d::LETHAL_OBSTACLE;
      touch(it->first.x, it->first.y, min_x, min_y, max_x, max_y);
    }
  }

  std::unordered_set<volume_grid::occupany_cell>::iterator cell;
  for (cell = cleared_cells.begin(); cell != cleared_cells.end(); ++cell) {
    touch(cell->x, cell->y, min_x, min_y, max_x, max_y);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateBounds(double robot_x, double robot_y,
                                            double robot_yaw, double* min_x,
                                            double* min_y, double* max_x,
                                            double* max_y)
/*****************************************************************************/
{
  // grabs new max bounds for the costmap
  if (!_enabled) {
    return;
  }

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  // Steve's Note June 22, 2018
  // I dislike this necessity, I can't remove the master grid's knowledge about
  // STVL on the fly so I have play games with the API even though this isn't
  // really a rolling plugin implementation. It works, but isn't ideal.
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - static_cast<double>(getSizeInMetersX()) / 2,
                 robot_y - static_cast<double>(getSizeInMetersY()) / 2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);
  _voxel_grid->SetRobotPose(robot_x, robot_y, robot_yaw);

  bool current = true;
  std::vector<observation::MeasurementReading> marking_observations,
      clearing_observations;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  current = GetMarkingObservations(marking_observations) && current;
  current = GetClearingObservations(clearing_observations) && current;
  current = GetDynamicObstacleReadings(dynamic_obstacle_readings) && current;
  current = GetRobotMotionReadings(robot_motion_readings) && current;
  ObservationsResetAfterReading();
  current_ = current;

  std::unordered_set<volume_grid::occupany_cell> cleared_cells;

  // navigation mode: clear observations, mapping mode: save maps and publish
  if (!_mapping_mode) {
    _voxel_grid->ClearFrustums(clearing_observations, cleared_cells,
                               dynamic_obstacle_readings,
                               robot_motion_readings);
  } else if (ros::Time::now() - _last_map_save_time > _map_save_duration) {
    _last_map_save_time = ros::Time::now();
    time_t rawtime;
    struct tm* timeinfo;
    char time_buffer[100];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 100, "%F-%r", timeinfo);

    spatio_temporal_voxel_layer::SaveGrid srv;
    srv.request.file_name.data = time_buffer;
    SaveGridCallback(srv.request, srv.response);
  }

  // mark observations
  _voxel_grid->Mark(marking_observations);

  // update the ROS Layered Costmap
  UpdateROSCostmap(min_x, min_y, max_x, max_y, cleared_cells);

  // publish point cloud in navigation mode
  bool is_time_limit_reached =
      ros::Time::now() > last_publish_time_ + publish_voxel_map_period_;
  if (_publish_voxels && !_mapping_mode && is_time_limit_reached) {
    last_publish_time_ = ros::Time::now();
    sensor_msgs::PointCloud2::Ptr pc2(new sensor_msgs::PointCloud2());
    _voxel_grid->GetOccupancyPointCloud(pc2);
    pc2->header.frame_id = _global_frame;
    pc2->header.stamp = ros::Time::now();
    _voxel_pub.publish(*pc2);
  }

  // update footprint
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::SaveGridCallback(
    spatio_temporal_voxel_layer::SaveGrid::Request& req,
    spatio_temporal_voxel_layer::SaveGrid::Response& resp)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  double map_size_bytes;

  if (_voxel_grid->SaveGrid(req.file_name.data, map_size_bytes)) {
    ROS_INFO(
        "SpatioTemporalVoxelGrid: Saved %s grid! Has memory footprint of %f "
        "bytes.",
        req.file_name.data.c_str(), map_size_bytes);
    resp.map_size_bytes = map_size_bytes;
    resp.status = true;
    return true;
  }

  ROS_WARN("SpatioTemporalVoxelGrid: Failed to save grid.");
  resp.status = false;
  return false;
}

void SpatioTemporalVoxelLayer::PublishBlindSpotPoint(
    const std::optional<openvdb::Vec3d>& world_coord) {
  visualization_msgs::Marker blind_spot_point;
  blind_spot_point.header.frame_id = _global_frame;
  blind_spot_point.header.stamp = ros::Time::now();
  blind_spot_point.ns = "blind_spot_point";
  blind_spot_point.id = 0;

  if (!world_coord) {
    blind_spot_point.action = visualization_msgs::Marker::DELETE;
  } else {
    blind_spot_point.type = visualization_msgs::Marker::SPHERE;
    blind_spot_point.action = visualization_msgs::Marker::ADD;
    blind_spot_point.pose.position.x = world_coord->x();
    blind_spot_point.pose.position.y = world_coord->y();
    blind_spot_point.pose.position.z = world_coord->z();
    blind_spot_point.scale.x = 0.2;
    blind_spot_point.scale.y = 0.2;
    blind_spot_point.scale.z = 0.2;
    blind_spot_point.color.a = 1.0;
    blind_spot_point.color.r = 1.0;
    blind_spot_point.color.g = 0.0;
    blind_spot_point.color.b = 1.0;
  }
  _blind_spot_pub.publish(blind_spot_point);
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::CheckBlindSpotCallback(
    spatio_temporal_voxel_layer::CheckBlindSpot::Request& req,
    spatio_temporal_voxel_layer::CheckBlindSpot::Response& resp)
/*****************************************************************************/
{
  if (req.point.header.frame_id.empty()) {
    ROS_ERROR("CheckBlindSpot: Request point is missing a frame_id.");
    resp.success = false;
    resp.msg = "no_frame_id";
    return true;
  }
  if (req.tolerance < 0.0) {
    ROS_ERROR("CheckBlindSpot: Request tolerance is negative.");
    resp.success = false;
    resp.msg = "negative_tolerance";
    return true;
  }
  if (req.tolerance > 5.0) {
    ROS_ERROR("CheckBlindSpot: Request tolerance is unreasonably large.");
    resp.success = false;
    resp.msg = "unreasonable_tolerance";
    return true;
  }
  if (req.min_height < 0.0) {
    ROS_ERROR("CheckBlindSpot: Request min_height is negative.");
    resp.success = false;
    resp.msg = "negative_min_height";
    return true;
  }
  if (req.max_height < req.min_height) {
    ROS_ERROR("CheckBlindSpot: Request max_height is less than min_height.");
    resp.success = false;
    resp.msg = "max_height_less_than_min_height";
    return true;
  }

  geometry_msgs::PointStamped global_point;
  try {
    tf_buffer_.transform(req.point, global_point, _global_frame,
                         ros::Duration(0.5));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("CheckBlindSpot: Failed to transform pose to global frame: %s",
              ex.what());
    resp.success = false;
    resp.msg = "transform_failed";
    return true;
  }

  // Extract XY position from the query
  const double query_x = global_point.point.x;
  const double query_y = global_point.point.y;

  // 2. Define the World-Space Query Region (BBox)
  openvdb::Vec3d min_world_corner(query_x - req.tolerance,
                                  query_y - req.tolerance, req.min_height);
  openvdb::Vec3d max_world_corner(query_x + req.tolerance,
                                  query_y + req.tolerance, req.max_height);

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  std::optional<openvdb::Vec3d> blind_spot_point =
      _voxel_grid->CheckBox(min_world_corner, max_world_corner);
  PublishBlindSpotPoint(blind_spot_point);
  if (!blind_spot_point) {
    resp.success = true;
    resp.is_blind_spot = false;
    resp.msg = "no_occupied_voxel";
    return true;
  }
  resp.success = true;
  resp.closest_point.point.x = blind_spot_point->x();
  resp.closest_point.point.y = blind_spot_point->y();
  resp.closest_point.point.z = blind_spot_point->z();
  resp.closest_point.header.frame_id = _global_frame;
  resp.is_blind_spot = true;
  resp.msg = "occupied_voxel";
  return true;
}

};  // namespace spatio_temporal_voxel_layer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer,
                       costmap_2d::Layer);
