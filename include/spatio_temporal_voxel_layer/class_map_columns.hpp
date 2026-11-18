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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_COLUMNS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_COLUMNS_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "spatio_temporal_voxel_layer/voxel_class.hpp"

namespace volume_grid {

struct VoxelColumn {
  int32_t kx;
  int32_t ky;
  bool operator==(const VoxelColumn& other) const {
    return kx == other.kx && ky == other.ky;
  }
  bool operator!=(const VoxelColumn& other) const { return !(*this == other); }
};

using ClassMask = uint16_t;
static_assert(kVoxelClassCount <= std::numeric_limits<ClassMask>::digits,
              "every VoxelClass needs a bit in the class mask");

constexpr ClassMask ToClassBit(VoxelClass cls) {
  return static_cast<ClassMask>(1u << static_cast<int32_t>(cls));
}

struct ColumnClassMask {
  VoxelColumn column;
  ClassMask mask;
};

// A column repeats up to once per OpenVDB leaf; readers OR the repeats.
using ClassMapColumns = std::vector<ColumnClassMask>;

}  // namespace volume_grid

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_CLASS_MAP_COLUMNS_HPP_
