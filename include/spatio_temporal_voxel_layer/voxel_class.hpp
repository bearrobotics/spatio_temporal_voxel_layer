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

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_VOXEL_CLASS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER_VOXEL_CLASS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace volume_grid {

// Every voxel in the STVL grid carries a VoxelClass (generic unless a source
// marked it otherwise). Each class has a VoxelClassPolicy holding how long the
// voxel lives and which clearing paths may remove it. Policies are looked up
// per voxel from a VoxelClassTable, validated and built once at startup.

// The class grid stores this enum as its Int32 value. Generic voxels are left
// unstored to keep that grid sparse, and its background is 0. So an unstored
// voxel reads back as kGeneric, which is why kGeneric must stay 0.
enum class VoxelClass : int32_t {
  kGeneric = 0,
  kCliff = 1,
  kCount,
};

constexpr std::size_t kVoxelClassCount =
    static_cast<std::size_t>(VoxelClass::kCount);

inline int32_t ToClassId(VoxelClass cls) { return static_cast<int32_t>(cls); }

namespace detail {
// Logs the corrupt id and returns kGeneric; out-of-line to keep it cold.
VoxelClass InvalidIdToGeneric(int32_t raw);
}  // namespace detail

// An out-of-range (corrupt) id is treated as generic rather than trusted.
inline VoxelClass ToVoxelClass(int32_t raw) {
  if (raw >= 0 && raw < static_cast<int32_t>(kVoxelClassCount)) {
    return static_cast<VoxelClass>(raw);
  }
  return detail::InvalidIdToGeneric(raw);
}

std::string ToClassName(VoxelClass cls);
std::optional<VoxelClass> ParseVoxelClass(const std::string& name);

// One row of the class table: how a voxel of this class decays and clears.
struct VoxelClassPolicy {
  // Lifetime after the last qualifying mark. 0 leaves the row invalid.
  double decay_seconds = 0.0;
  int32_t priority = 0;  // higher wins on a conflicting mark; unique

  // Separate observations needed to promote a voxel to this class.
  int32_t confirmation_frames = 1;
  // The longest gap between observations that keeps the count.
  double confirmation_window_seconds = 0.0;

  bool cleared_by_frustums = true;
  bool cleared_by_dynamic_obstacles = true;
  bool cleared_by_footprint_clear = true;
  bool cleared_by_front_blind_spot = true;
  bool decays_in_inter_sensor_prism = true;

  // Applies to configured rows only; the generic row is seeded from the
  // layer's voxel_decay and skips it.
  [[nodiscard]] bool IsValid() const;
};

// Immutable per-class policy lookup, built once at startup.
class VoxelClassTable {
 public:
  struct Row {
    VoxelClass voxel_class = VoxelClass::kGeneric;
    VoxelClassPolicy policy;
  };

  struct Config {
    double generic_decay_seconds;  // from the layer's voxel_decay param
    std::vector<Row> rows;         // non-generic classes only
  };

  // Builds and validates the table; nullopt (logged) if the config is rejected.
  static std::optional<VoxelClassTable> Create(const Config& config);

  // Table with only the generic row; cannot fail, unlike Create.
  static VoxelClassTable CreateGenericOnly(double generic_decay_seconds);

  [[nodiscard]] const VoxelClassPolicy& GetPolicy(VoxelClass cls) const {
    if (static_cast<std::size_t>(cls) >= kVoxelClassCount) {
      cls = detail::InvalidIdToGeneric(ToClassId(cls));
    }
    return policies_[static_cast<std::size_t>(cls)];
  }

  // Generic always exists; Create gives every configured class a priority
  // above generic's.
  [[nodiscard]] bool IsConfigured(VoxelClass cls) const {
    return cls == VoxelClass::kGeneric || GetPolicy(cls).priority >= 1;
  }

 private:
  VoxelClassTable() = default;

  std::array<VoxelClassPolicy, kVoxelClassCount> policies_;
};

}  // namespace volume_grid

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_VOXEL_CLASS_HPP_
