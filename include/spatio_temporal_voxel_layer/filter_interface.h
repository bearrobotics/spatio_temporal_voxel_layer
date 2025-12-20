#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FILTER_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_FILTER_H_

#include <pcl_conversions/pcl_conversions.h>

namespace spatio_temporal_voxel_layer {
class Filter {
 public:
  virtual ~Filter() = default;
  virtual void ApplyFilter(pcl::PCLPointCloud2::Ptr cloud_pcl) = 0;
};
}  // namespace spatio_temporal_voxel_layer
#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_FILTER_H_
