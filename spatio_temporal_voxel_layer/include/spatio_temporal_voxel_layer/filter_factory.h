#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FILTER_FACTORY_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_FILTER_FACTORY_H_

#include <memory>

#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/filter_interface.h"

namespace spatio_temporal_voxel_layer {

/// @brief Factory class for creating point cloud filters based on ROS
/// parameters.
///
/// FilterFactory reads the "type" parameter from the ROS parameter server
/// and instantiates the corresponding filter implementation.
class FilterFactory {
 public:
  /// @brief Creates a filter based on the "type" parameter in the given
  /// NodeHandle.
  ///
  /// Supported filter types:
  /// - "none": No filtering (not implemented)
  /// - "voxel": Voxel grid downsampling (not implemented)
  /// - "passthrough": Pass-through filter (not implemented)
  /// - "noise": NoiseFilter with statistical outlier removal and height
  /// filtering
  ///
  /// @param nh NodeHandle containing the filter configuration parameters.
  ///           Must have a "type" parameter specifying the filter type.
  /// @return A unique pointer to the created filter, or nullptr if:
  ///         - The "type" parameter is missing or empty
  ///         - The filter type is unknown
  ///         - The filter type is not implemented
  ///         - Filter-specific configuration is invalid
  static std::unique_ptr<Filter> CreateFilter(ros::NodeHandle& nh);

 private:
  static std::unique_ptr<Filter> CreateNoFilter(ros::NodeHandle& nh);
  static std::unique_ptr<Filter> CreateVoxelGridFilter(ros::NodeHandle& nh);
  static std::unique_ptr<Filter> CreatePassThroughFilter(ros::NodeHandle& nh);
  static std::unique_ptr<Filter> CreateNoiseFilter(ros::NodeHandle& nh);
};

}  // namespace spatio_temporal_voxel_layer
#endif
