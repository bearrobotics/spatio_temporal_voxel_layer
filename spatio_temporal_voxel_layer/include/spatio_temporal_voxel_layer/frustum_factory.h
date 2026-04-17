#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTRUM_FACTORY_H
#define SPATIO_TEMPORAL_VOXEL_LAYER_FRUSTRUM_FACTORY_H

#include <functional>
#include <optional>

#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/frustum.hpp"

/**
 * @brief Factory-of-factories for frustum objects.
 *
 * Creates factory functions that produce frustum instances. Configuration is
 * loaded from the ROS parameter server at factory-creation time, enabling
 * lightweight frustum instantiation later. Supports different frustum types
 * for various sensor models.
 */
class FrustumFactoryFactory {
 public:
  using FrustumFactory = std::function<std::unique_ptr<geometry::Frustum>()>;

  /**
   * @brief Creates a frustum factory function based on ROS parameters.
   *
   * Reads the "type" parameter from the provided NodeHandle and returns
   * a function that creates the appropriate frustum type.
   *
   * @param nh ROS NodeHandle pointing to the namespace containing frustum
   *           config.
   * @return FrustumFactory function that creates frustums, or nullptr on error.
   *
   * @note Supported types: "3d_camera"
   */
  static FrustumFactory CreateFrustumFactory(ros::NodeHandle& nh);

 private:
  /**
   * @brief Creates a factory function for frustum objects.
   *
   * @param nh ROS NodeHandle containing depth camera frustum parameters.
   * @return FrustumFactory function, or nullptr if config loading fails.
   */
  template <class TFrustumFactory>
  static FrustumFactory CreateGenericFrustumFactory(ros::NodeHandle& nh) {
    std::optional<typename TFrustumFactory::Config> config =
        TFrustumFactory::Config::Load(nh);
    if (!config) {
      ROS_ERROR("Failed to load %s config", typeid(TFrustumFactory).name());
      return nullptr;
    }
    return [config]() {
      return std::unique_ptr<geometry::Frustum>(new TFrustumFactory(*config));
    };
  }
};

#endif
