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

#include "spatio_temporal_voxel_layer/noise_filter.h"

#include <boost/bind.hpp>

#include "bearlib/ros/param_loader.h"

namespace spatio_temporal_voxel_layer {

std::optional<NoiseFilter::Config> NoiseFilter::Config::Load(
    ros::NodeHandle& nh) {
  using namespace bear::lib::ros;
  Config config;

  auto min_obstacle_height =
      LoadOptionalParam<float>(nh, "min_obstacle_height");
  auto max_obstacle_height =
      LoadOptionalParam<float>(nh, "max_obstacle_height");
  auto mean_k = LoadOptionalParam<int>(nh, "mean_k");
  auto std_dev_mul_thresh = LoadOptionalParam<float>(nh, "std_dev_mul_thresh");
  auto voxel_min_points = LoadOptionalParam<int>(nh, "voxel_min_points");

  if (!min_obstacle_height || !max_obstacle_height || !mean_k ||
      !std_dev_mul_thresh || !voxel_min_points) {
    ROS_ERROR("Missing required parameters for NoiseFilter.");
    return std::nullopt;
  }

  config.min_obstacle_height = *min_obstacle_height;
  config.max_obstacle_height = *max_obstacle_height;
  config.mean_k = *mean_k;
  config.std_dev_mul_thresh = *std_dev_mul_thresh;
  config.voxel_min_points = *voxel_min_points;

  return config;
}

bool NoiseFilter::Config::IsValid() const {
  if (min_obstacle_height < 0.0f || max_obstacle_height < min_obstacle_height) {
    ROS_ERROR("Invalid obstacle height parameters for NoiseFilter.");
    return false;
  }
  if (mean_k < 1) {
    ROS_ERROR("mean_k must be at least 1 for NoiseFilter.");
    return false;
  }
  if (std_dev_mul_thresh < 0.0f) {
    ROS_ERROR("std_dev_mul_thresh must be non-negative for NoiseFilter.");
    return false;
  }
  if (voxel_min_points < 1) {
    ROS_ERROR("voxel_min_points must be at least 1 for NoiseFilter.");
    return false;
  }
  return true;
}

std::unique_ptr<NoiseFilter> NoiseFilter::Create(
    const NoiseFilter::Config& config, ros::NodeHandle& nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  return std::unique_ptr<NoiseFilter>(new NoiseFilter(config, nh));
}

void NoiseFilter::ApplyFilter(pcl::PCLPointCloud2::Ptr cloud_pcl) {
  std::lock_guard<std::mutex> lock(mutex_);

  bool is_empty = cloud_pcl->width == 0 || cloud_pcl->height == 0 ||
                  cloud_pcl->data.empty();
  if (!is_empty) {
    statistical_outlier_filter_.setInputCloud(cloud_pcl);
    statistical_outlier_filter_.filter(*cloud_pcl);
  }
  pass_through_filter_.setInputCloud(cloud_pcl);
  pass_through_filter_.filter(*cloud_pcl);
}

NoiseFilter::NoiseFilter(const NoiseFilter::Config& config, ros::NodeHandle& nh)
    : config_(config), nh_(nh) {
  UpdateFilterParameters();

  dynamic_reconfigure_server_ =
      std::make_unique<dynamic_reconfigure::Server<NoiseFilterConfig>>(nh_);
  dynamic_reconfigure::Server<NoiseFilterConfig>::CallbackType cb =
      boost::bind(&NoiseFilter::DynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(cb);
}

void NoiseFilter::DynamicReconfigureCallback(NoiseFilterConfig& config,
                                             uint32_t level) {
  Config new_config;
  new_config.min_obstacle_height =
      static_cast<float>(config.min_obstacle_height);
  new_config.max_obstacle_height =
      static_cast<float>(config.max_obstacle_height);
  new_config.mean_k = config.mean_k;
  new_config.std_dev_mul_thresh = static_cast<float>(config.std_dev_mul_thresh);
  new_config.voxel_min_points = config.voxel_min_points;

  if (!new_config.IsValid()) {
    ROS_WARN(
        "NoiseFilter: Invalid dynamic reconfigure parameters, ignoring "
        "update.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  config_ = new_config;
  UpdateFilterParameters();
}

void NoiseFilter::UpdateFilterParameters() {
  statistical_outlier_filter_.setKeepOrganized(false);
  statistical_outlier_filter_.setMeanK(config_.mean_k);
  statistical_outlier_filter_.setStddevMulThresh(config_.std_dev_mul_thresh);

  pass_through_filter_.setKeepOrganized(false);
  pass_through_filter_.setFilterFieldName("z");
  pass_through_filter_.setFilterLimits(config_.min_obstacle_height,
                                       config_.max_obstacle_height);
}

}  // namespace spatio_temporal_voxel_layer
