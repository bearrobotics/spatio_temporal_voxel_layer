/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2026, Bear Robotics, Inc.
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

#include "spatio_temporal_voxel_layer/class_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace volume_grid {

bool ClassMapBuilder::ColumnBox::Contains(const VoxelColumn& column) const {
  return column.kx >= lower.kx && column.kx <= upper.kx &&
         column.ky >= lower.ky && column.ky <= upper.ky;
}

void ClassMapBuilder::ColumnBox::Include(const VoxelColumn& column) {
  lower.kx = std::min(lower.kx, column.kx);
  lower.ky = std::min(lower.ky, column.ky);
  upper.kx = std::max(upper.kx, column.kx);
  upper.ky = std::max(upper.ky, column.ky);
}

ClassMapBuilder::ClassMapBuilder(double voxel_size, double window_width_m,
                                 const VoxelClassTable& class_table)
    : voxel_size_(voxel_size),
      window_width_m_(window_width_m),
      // +2: float error can widen both window edges by one column.
      max_window_width_columns_(
          static_cast<std::size_t>(std::ceil(window_width_m / voxel_size)) +
          2) {
  for (std::size_t i = 0; i < kVoxelClassCount; ++i) {
    const VoxelClass cls = static_cast<VoxelClass>(i);
    class_names_[i] =
        class_table.GetPolicy(cls).publish_class_map ? ToClassName(cls) : "";
  }
}

void ClassMapBuilder::Fill(const ClassMapColumns& columns, double robot_x,
                           double robot_y, bear_msgs::ClassMap* map) const {
  map->mask.reserve(max_window_width_columns_ * max_window_width_columns_);
  const ColumnBox window = ComputeWindow(robot_x, robot_y);
  WriteFixedFields(map);
  WriteWindow(window, map);
  WriteBox(columns, ComputeBox(columns, window), map);
}

ClassMapBuilder::ColumnBox ClassMapBuilder::ComputeWindow(
    double robot_x, double robot_y) const {
  const double half_window_width = window_width_m_ / 2.0;
  const auto first_column = [this](double world_coord) {
    return static_cast<int32_t>(std::floor(world_coord / voxel_size_));
  };
  const auto last_column = [this](double world_coord) {
    return static_cast<int32_t>(std::ceil(world_coord / voxel_size_)) - 1;
  };
  return {{first_column(robot_x - half_window_width),
           first_column(robot_y - half_window_width)},
          {last_column(robot_x + half_window_width),
           last_column(robot_y + half_window_width)}};
}

std::optional<ClassMapBuilder::ColumnBox> ClassMapBuilder::ComputeBox(
    const ClassMapColumns& columns, const ColumnBox& window) {
  std::optional<ColumnBox> box;
  for (const ColumnClassMask& entry : columns) {
    if (!window.Contains(entry.column)) {
      continue;
    }
    if (box) {
      box->Include(entry.column);
    } else {
      box = ColumnBox{entry.column, entry.column};
    }
  }
  return box;
}

void ClassMapBuilder::WriteFixedFields(bear_msgs::ClassMap* map) const {
  map->info.resolution = static_cast<float>(voxel_size_);
  map->info.origin.orientation.x = 0.0;
  map->info.origin.orientation.y = 0.0;
  map->info.origin.orientation.z = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->info.origin.position.z = 0.0;
  map->class_names.assign(class_names_.begin(), class_names_.end());
}

void ClassMapBuilder::WriteWindow(const ColumnBox& window,
                                  bear_msgs::ClassMap* map) const {
  map->window_min.x = window.lower.kx * voxel_size_;
  map->window_min.y = window.lower.ky * voxel_size_;
  map->window_min.z = 0.0;
  map->window_max.x = (window.upper.kx + 1) * voxel_size_;
  map->window_max.y = (window.upper.ky + 1) * voxel_size_;
  map->window_max.z = 0.0;
}

void ClassMapBuilder::WriteBox(const ClassMapColumns& columns,
                               const std::optional<ColumnBox>& box,
                               bear_msgs::ClassMap* map) const {
  if (!box) {
    map->info.width = 0;
    map->info.height = 0;
    map->info.origin.position.x = 0.0;
    map->info.origin.position.y = 0.0;
    map->mask.clear();
    return;
  }
  const uint32_t width =
      static_cast<uint32_t>(box->upper.kx - box->lower.kx) + 1;
  const uint32_t height =
      static_cast<uint32_t>(box->upper.ky - box->lower.ky) + 1;
  map->info.width = width;
  map->info.height = height;
  map->info.origin.position.x = box->lower.kx * voxel_size_;
  map->info.origin.position.y = box->lower.ky * voxel_size_;
  map->mask.assign(static_cast<std::size_t>(width) * height, 0);
  for (const ColumnClassMask& entry : columns) {
    if (!box->Contains(entry.column)) {
      continue;
    }
    const std::size_t index =
        static_cast<std::size_t>(entry.column.ky - box->lower.ky) * width +
        static_cast<std::size_t>(entry.column.kx - box->lower.kx);
    map->mask[index] |= entry.mask;
  }
}

}  // namespace volume_grid
