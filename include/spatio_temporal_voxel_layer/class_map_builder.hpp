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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_BUILDER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_BUILDER_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include "bear_msgs/ClassMap.h"
#include "spatio_temporal_voxel_layer/class_map_columns.hpp"
#include "spatio_temporal_voxel_layer/voxel_class.hpp"

namespace volume_grid {

// Builds the class map box around the recorded columns inside the window.
class ClassMapBuilder {
 public:
  ClassMapBuilder(double voxel_size, double window_width_m,
                  const VoxelClassTable& class_table);

  // Leaves header and info.map_load_time to the caller.
  void Fill(const ClassMapColumns& columns, double robot_x, double robot_y,
            bear_msgs::ClassMap* map) const;

 private:
  // Inclusive range of voxel columns.
  struct ColumnBox {
    VoxelColumn lower;
    VoxelColumn upper;

    bool Contains(const VoxelColumn& column) const;
    void Include(const VoxelColumn& column);
  };

  ColumnBox ComputeWindow(double robot_x, double robot_y) const;
  static std::optional<ColumnBox> ComputeBox(const ClassMapColumns& columns,
                                             const ColumnBox& window);
  void WriteFixedFields(bear_msgs::ClassMap* map) const;
  void WriteWindow(const ColumnBox& window, bear_msgs::ClassMap* map) const;
  void WriteBox(const ClassMapColumns& columns,
                const std::optional<ColumnBox>& box,
                bear_msgs::ClassMap* map) const;

  double voxel_size_;
  double window_width_m_;
  std::size_t max_window_width_columns_;
  std::array<std::string, kVoxelClassCount> class_names_;
};

}  // namespace volume_grid

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_BUILDER_HPP_
