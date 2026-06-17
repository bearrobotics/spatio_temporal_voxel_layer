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
 * Author: Hoon (Seung-Hun) Han (seunghun.han@bearrobotics.ai)
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_PASSTHROUGH_FILTER_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_PASSTHROUGH_FILTER_H_

#include "spatio_temporal_voxel_layer/filter_interface.h"

namespace spatio_temporal_voxel_layer {

/// No-op filter for use when upstream (e.g. Sensor Pipeline V2) is already the
/// single source of point cloud filtering.
class PassthroughFilter : public Filter {
 public:
  void ApplyFilter(pcl::PCLPointCloud2::Ptr cloud_pcl) override {}
};

}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_PASSTHROUGH_FILTER_H_
