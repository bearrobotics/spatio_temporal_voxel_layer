#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_NOISE_FILTER_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_NOISE_FILTER_H_

#include <dynamic_reconfigure/server.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <mutex>
#include <optional>

#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/NoiseFilterConfig.h"
#include "spatio_temporal_voxel_layer/filter_interface.h"

namespace spatio_temporal_voxel_layer {

/// @brief Point cloud filter combining statistical outlier removal and height
/// filtering.
///
/// NoiseFilter applies two filtering stages to incoming point clouds:
/// 1. Statistical outlier removal to eliminate noise points
/// 2. Pass-through filter to keep only points within a specified height range
///
/// The filter supports dynamic reconfiguration of parameters at runtime.
class NoiseFilter : public Filter {
 public:
  /// @brief Configuration parameters for NoiseFilter.
  struct Config {
    [[nodiscard]] bool IsValid() const;
    /// @brief Loads filter configuration from the ROS parameter server.
    ///
    /// Expected parameters under the given NodeHandle:
    /// - min_obstacle_height (float): Minimum height for obstacles
    /// - max_obstacle_height (float): Maximum height for obstacles
    /// - mean_k (int): Number of neighbors for statistical analysis
    /// - std_dev_mul_thresh (float): Standard deviation multiplier
    /// - voxel_min_points (int): Minimum points per voxel
    ///
    /// @param nh NodeHandle containing the configuration parameters.
    /// @return The loaded configuration, or std::nullopt if any required
    ///         parameter is missing.
    static std::optional<Config> Load(ros::NodeHandle& nh);

    /// Minimum obstacle height in meters. Points below this are removed.
    float min_obstacle_height = 0.0f;
    /// Maximum obstacle height in meters. Points above this are removed.
    float max_obstacle_height = 0.0f;
    /// Number of nearest neighbors for statistical outlier analysis.
    int mean_k = 0;
    /// Standard deviation multiplier threshold for outlier detection.
    float std_dev_mul_thresh = 0.0f;
    /// Minimum points per voxel (reserved for future use).
    int voxel_min_points = 0;
  };

  /// @brief Creates a NoiseFilter instance with the given configuration.
  ///
  /// @param config Filter configuration parameters.
  /// @param nh NodeHandle for dynamic reconfigure server.
  /// @return A unique pointer to the created filter, or nullptr if the
  ///         configuration is invalid. Invalid configurations include:
  ///         - min_obstacle_height < 0
  ///         - max_obstacle_height < min_obstacle_height
  ///         - mean_k < 1
  ///         - std_dev_mul_thresh < 0
  ///         - voxel_min_points < 1
  static std::unique_ptr<NoiseFilter> Create(const Config& config,
                                             ros::NodeHandle& nh);

  /// @brief Filters the input point cloud in-place.
  ///
  /// Applies statistical outlier removal followed by height-based filtering.
  /// Empty clouds (width=0, height=0, or empty data) are handled gracefully.
  /// This method is thread-safe.
  ///
  /// @param cloud_pcl Point cloud to filter. Modified in-place.
  void ApplyFilter(pcl::PCLPointCloud2::Ptr cloud_pcl) override;

 private:
  explicit NoiseFilter(const Config& config, ros::NodeHandle& nh);
  void DynamicReconfigureCallback(NoiseFilterConfig& config, uint32_t level);
  void UpdateFilterParameters();

  ros::NodeHandle nh_;
  std::mutex mutex_;
  Config config_;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid_filter_;
  pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter_;
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>
      statistical_outlier_filter_;
  std::unique_ptr<dynamic_reconfigure::Server<NoiseFilterConfig>>
      dynamic_reconfigure_server_;
};

}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_NOISE_FILTER_H_
