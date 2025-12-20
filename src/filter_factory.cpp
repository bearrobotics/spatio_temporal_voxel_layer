#include "spatio_temporal_voxel_layer/filter_factory.h"

#include "bearlib/ros/param_loader.h"
#include "spatio_temporal_voxel_layer/noise_filter.h"

namespace spatio_temporal_voxel_layer {

std::unique_ptr<Filter> FilterFactory::CreateFilter(ros::NodeHandle& nh) {
  std::string filter_type =
      bear::lib::ros::LoadRequiredParam<std::string>(nh, "type");
  if (filter_type.empty()) {
    ROS_ERROR("Filter type parameter is empty");
    return nullptr;
  }
  if (filter_type == "none") {
    return CreateNoFilter(nh);
  }
  if (filter_type == "voxel") {
    return CreateVoxelGridFilter(nh);
  }
  if (filter_type == "passthrough") {
    return CreatePassThroughFilter(nh);
  }
  if (filter_type == "noise") {
    return CreateNoiseFilter(nh);
  }
  ROS_ERROR_STREAM("Unknown filter type: " << filter_type);
  return nullptr;
}

std::unique_ptr<Filter> FilterFactory::CreateNoFilter(ros::NodeHandle& nh) {
  ROS_ERROR("NoFilter is not implemented yet");
  return nullptr;
}

std::unique_ptr<Filter> FilterFactory::CreateVoxelGridFilter(
    ros::NodeHandle& nh) {
  ROS_ERROR("VoxelGridFilter is not implemented yet");
  return nullptr;
}

std::unique_ptr<Filter> FilterFactory::CreatePassThroughFilter(
    ros::NodeHandle& nh) {
  ROS_ERROR("PassThroughFilter is not implemented yet");
  return nullptr;
}

std::unique_ptr<Filter> FilterFactory::CreateNoiseFilter(ros::NodeHandle& nh) {
  auto config = NoiseFilter::Config::Load(nh);
  if (!config) {
    ROS_ERROR("Failed to load NoiseFilter configuration.");
    return nullptr;
  }
  return NoiseFilter::Create(*config, nh);
}

}  // namespace spatio_temporal_voxel_layer
