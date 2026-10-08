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

#include "spatio_temporal_voxel_layer/voxel_class.hpp"

#include <ros/ros.h>

#include <array>
#include <cmath>
#include <set>

namespace volume_grid {

namespace {

// Indexed by VoxelClass value; the static_assert catches a class added
// without a name.
constexpr std::array kClassNames = {"generic", "cliff"};
static_assert(kClassNames.size() == kVoxelClassCount,
              "every VoxelClass needs a name");

}  // namespace

bool VoxelClassPolicy::IsValid() const {
  if (!std::isfinite(decay_seconds) || decay_seconds <= 0.0 || priority < 1 ||
      confirmation_frames < 1) {
    return false;
  }
  // A single confirmation frame is the same as no confirmation.
  if (confirmation_frames == 1) {
    return true;
  }
  // The window must be positive, and neither infinite nor NaN.
  return std::isfinite(confirmation_window_seconds) &&
         confirmation_window_seconds > 0.0;
}

VoxelClassTable VoxelClassTable::CreateGenericOnly(
    double generic_decay_seconds) {
  VoxelClassPolicy generic;
  generic.decay_seconds = generic_decay_seconds;
  generic.priority = 0;

  // An unconfigured class resolves to the generic policy.
  VoxelClassTable table;
  table.policies_.fill(generic);
  return table;
}

std::optional<VoxelClassTable> VoxelClassTable::Create(const Config& config) {
  VoxelClassTable table = CreateGenericOnly(config.generic_decay_seconds);

  std::set<int32_t> seen_priorities = {0};  // generic occupies priority 0
  std::set<std::size_t> seen_classes = {
      static_cast<std::size_t>(VoxelClass::kGeneric)};

  for (const Row& row : config.rows) {
    const std::size_t idx = static_cast<std::size_t>(row.voxel_class);
    const std::string name = ToClassName(row.voxel_class);

    if (row.voxel_class == VoxelClass::kGeneric || idx >= kVoxelClassCount) {
      ROS_ERROR_STREAM("voxel_classes: '" << name
                                          << "' is not a configurable class "
                                             "(generic is implicit).");
      return std::nullopt;
    }
    if (!seen_classes.insert(idx).second) {
      ROS_ERROR_STREAM("voxel_classes: class '" << name
                                                << "' configured more than "
                                                   "once.");
      return std::nullopt;
    }
    if (!row.policy.IsValid()) {
      ROS_ERROR_STREAM("voxel_classes: class '"
                       << name << "' has priority " << row.policy.priority
                       << ", decay " << row.policy.decay_seconds
                       << ", confirmation frames "
                       << row.policy.confirmation_frames << ", window "
                       << row.policy.confirmation_window_seconds
                       << ". Requires priority >= 1, a finite decay > 0, "
                          "frames >= 1, and a finite window > 0 when frames "
                          "> 1.");
      return std::nullopt;
    }
    if (!seen_priorities.insert(row.policy.priority).second) {
      ROS_ERROR_STREAM("voxel_classes: priority "
                       << row.policy.priority << " on class '" << name
                       << "' is already used by another class.");
      return std::nullopt;
    }
    table.policies_[idx] = row.policy;
  }

  return table;
}

std::string ToClassName(VoxelClass cls) {
  const std::size_t idx = static_cast<std::size_t>(cls);
  return idx < kClassNames.size() ? kClassNames[idx] : "unknown";
}

std::optional<VoxelClass> ParseVoxelClass(const std::string& name) {
  for (std::size_t i = 0; i < kClassNames.size(); ++i) {
    if (name == kClassNames[i]) {
      return static_cast<VoxelClass>(i);
    }
  }
  return std::nullopt;
}

VoxelClass detail::InvalidIdToGeneric(int32_t raw) {
  ROS_ERROR_THROTTLE(5.0,
                     "Read voxel class id %d outside [0, %zu); treating as "
                     "generic. Class grid may be corrupt.",
                     raw, kVoxelClassCount);
  return VoxelClass::kGeneric;
}

}  // namespace volume_grid
