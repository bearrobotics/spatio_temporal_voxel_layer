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

#ifndef NAVIGATION_SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_POLYGON_FRUSTUM_H
#define NAVIGATION_SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_POLYGON_FRUSTUM_H

#include "boost/geometry.hpp"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/clearing_frustum.h"

namespace geometry {
/**
 * @class Polygon
 * @brief A polygon clearing frustum for dynamic obstacle clearing.
 */
class Polygon : public IClearingFrustum {
 public:
  /**
   * @brief Check if a point is inside the polygon.
   * @param point The 3D point to check (only x,y are used).
   * @return True if the point is inside the polygon, false otherwise.
   */
  bool IsInside(const openvdb::Vec3d& point) const override {
    DynamicObstacleReading::BoostPoint boost_point{point[0], point[1]};
    return boost::geometry::within(boost_point, polygon_);
  }

  /// The boost geometry polygon defining the clearing region.
  DynamicObstacleReading::Polygon polygon_;
};
}  // namespace geometry
#endif
