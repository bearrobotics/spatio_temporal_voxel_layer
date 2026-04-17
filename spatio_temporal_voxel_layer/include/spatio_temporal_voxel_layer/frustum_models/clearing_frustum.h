#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_CLEARING_FRUSTUM_H
#define SPATIO_TEMPORAL_VOXEL_LAYER_CLEARING_FRUSTUM_H
#include <openvdb/openvdb.h>
namespace geometry {
class IClearingFrustum {
 public:
  virtual ~IClearingFrustum() = default;

  /**
   * @brief Check if a point is inside the frustum.
   * @param point The point to check.
   * @return True if the point is inside the frustum, false otherwise.
   */
  virtual bool IsInside(const openvdb::Vec3d& point) const = 0;
};
}  // namespace geometry
#endif
