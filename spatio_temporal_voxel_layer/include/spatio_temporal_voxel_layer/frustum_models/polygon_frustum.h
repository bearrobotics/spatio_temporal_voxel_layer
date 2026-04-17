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
