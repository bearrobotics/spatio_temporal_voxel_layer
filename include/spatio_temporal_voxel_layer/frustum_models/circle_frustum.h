#ifndef NAVIGATION_SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_CIRCLE_FRUSTUM_H
#define NAVIGATION_SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTUM_MODELS_CIRCLE_FRUSTUM_H

#include "spatio_temporal_voxel_layer/frustum_models/clearing_frustum.h"

namespace geometry {
/**
 * @class Circle
 * @brief A circular clearing frustum for dynamic obstacle clearing.
 */
class Circle : public IClearingFrustum {
 public:
  /**
   * @brief Check if a point is inside the circle.
   * @param point The 3D point to check (only x,y are used).
   * @return True if the point is inside the circle, false otherwise.
   */
  [[nodiscard]] bool IsInside(const openvdb::Vec3d& point) const override {
    float dx = point[0] - center[0];
    float dy = point[1] - center[1];
    float distance_2 = (dx * dx) + (dy * dy);
    return distance_2 <= (radius * radius);
  }

  /// Center position of the circle in the map frame.
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  /// Radius of the clearing circle in meters.
  float radius = 0.0;
};
}  // namespace geometry
#endif
