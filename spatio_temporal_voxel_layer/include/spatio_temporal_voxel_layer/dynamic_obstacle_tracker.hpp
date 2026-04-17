#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_TRACKER_HPP
#define SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_TRACKER_HPP

#include <openvdb/openvdb.h>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "boost/geometry.hpp"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/circle_frustum.h"
#include "spatio_temporal_voxel_layer/frustum_models/clearing_frustum.h"
#include "spatio_temporal_voxel_layer/frustum_models/polygon_frustum.h"
#include "visualization_msgs/MarkerArray.h"

/**
 * @class DynamicObstacleTracker
 * @brief Tracks dynamic obstacles and generates clearing frustums for the
 *        spatio-temporal voxel layer.
 *
 * This class maintains a history of dynamic obstacle positions and velocities,
 * interpolating between readings to create clearing regions that prevent
 * spurious obstacle detection along the obstacle's trajectory.
 */
class DynamicObstacleTracker {
 public:
  using Circle = geometry::Circle;
  using Polygon = geometry::Polygon;
  using IClearingFrustum = geometry::IClearingFrustum;

  /**
   * @brief Configuration parameters for the dynamic obstacle tracker.
   */
  struct Config {
    /**
     * @brief Load configuration from ROS parameters.
     * @param nh ROS NodeHandle to load parameters from.
     * @return Config if all parameters were loaded, std::nullopt otherwise.
     */
    static std::optional<Config> LoadConfig(ros::NodeHandle& nh);

    /**
     * @brief Validate configuration parameter values.
     * @return True if all parameters are valid, false otherwise.
     */
    bool IsValid() const;

    /// Whether dynamic obstacle tracking is enabled.
    bool enable = false;
    /// Minimum velocity (m/s) for an obstacle to be considered dynamic.
    double activation_velocity_threshold = -1.0;
    /// Minimum distance (m) between readings to add a new point.
    double min_distance_between_readings_threshold = -1.0;
    /// Time (s) to project obstacle position forward along velocity vector.
    double interpolation_in_future = -1.0;
    /// Time (s) to project obstacle position backward along velocity vector.
    double interpolation_in_past = -1.0;
    /// Time (s) after which readings are considered stale and removed.
    double stale_time_threshold = -1.0;
    /// Factor to multiply obstacle radius for clearing circle inflation.
    double inflation_radius_factor = -1.0;
    /// Number of circles to interpolate between consecutive readings.
    int number_of_interpolation_circles = -1;
    /// Time window (s) for considering past readings when generating circles.
    double past_time_window = -1.0;
    /// Maximum probability for random_walk model to consider track enabled.
    double random_walk_probability_limit = -1.0;
    /// Duration (s) random_walk probability must stay below limit to enable.
    double seconds_since_last_random_walk = -1.0;
    /// Whether to publish visualization markers.
    bool publish_visualization = false;
    /// Maximum allowed obstacle radius (m). Tracks exceeding this are removed.
    double max_obstacle_radius = -1.0;
  };

  /**
   * @brief Stores the tracking history for a single dynamic obstacle.
   */
  struct TrackInfo {
    /// Whether this obstacle has exceeded the velocity threshold.
    bool enabled = false;
    /// History of tracked readings for this obstacle.
    std::vector<DynamicObstacleReading> readings;
    /// Maximum observed radius for this obstacle.
    float max_radius = 0.0;
    /// Last time random_walk probability exceeded the limit.
    ros::Time last_exceeded_random_walk_limit;
  };

  /**
   * @brief Factory function to create a DynamicObstacleTracker instance.
   * @param config Configuration parameters.
   * @param nh ROS NodeHandle for the tracker.
   * @return Unique pointer to the tracker, or nullptr if config is invalid.
   */
  static std::unique_ptr<DynamicObstacleTracker> Create(const Config& config,
                                                        ros::NodeHandle& nh);

  /**
   * @brief Generate interpolated circles between two obstacle positions.
   * @param begin_circle Starting circle position and radius.
   * @param end_circle Ending circle position and radius.
   * @param num_circles Number of circles to interpolate.
   * @param interpolated_circles Output vector to append generated circles.
   */
  void GenerateInterpolatedCircles(
      Circle begin_circle, Circle end_circle, int num_circles,
      std::vector<std::unique_ptr<Circle>>& interpolated_circles) const;

  /**
   * @brief Update tracking data for an existing obstacle.
   * @param it Iterator to the obstacle's entry in the track map.
   * @param reading New reading data for the obstacle.
   */
  void UpdateExistingReading(
      std::unordered_map<uint32_t, TrackInfo>::iterator& it,
      const DynamicObstacleReading& reading);

  /**
   * @brief Add a new obstacle to the tracking map.
   * @param reading Initial reading data for the new obstacle.
   */
  void AddNewReading(const DynamicObstacleReading& reading);

  /**
   * @brief Process obstacle readings and generate clearing frustums.
   *
   * This is the main entry point for the tracker. It updates the internal
   * tracking state with new readings, removes stale data, and generates
   * clearing frustums (circles and polygons) for use by the voxel layer.
   *
   * @param readings Vector of obstacle readings to process.
   * @return Vector of clearing frustums covering dynamic obstacle trajectories.
   */
  std::vector<std::unique_ptr<IClearingFrustum>>
  GenerateDynamicObstacleClearingFrustums(
      std::vector<DynamicObstacleReading>& readings);

  /**
   * @brief Generate clearing circles from all tracked obstacles.
   * @return Vector of circles covering tracked obstacle trajectories.
   */
  std::vector<std::unique_ptr<Circle>> GenerateCircles();

  /**
   * @brief Remove obstacles that haven't been updated within the stale
   * threshold.
   */
  void RemoveStaleReadings();

 private:
  /**
   * @brief Construct a DynamicObstacleTracker with the given configuration.
   * @param config Configuration parameters.
   * @param nh ROS NodeHandle for the tracker.
   */
  explicit DynamicObstacleTracker(const Config& config, ros::NodeHandle& nh);

  /**
   * @brief Compute the squared Euclidean distance between two 2D points.
   * @param a First point.
   * @param b Second point.
   * @return Squared distance between a and b.
   */
  static double SquaredDistance(const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b);

  /**
   * @brief Check if two points are within the minimum distance threshold.
   * @param a First point.
   * @param b Second point.
   * @return True if distance between a and b is less than the threshold.
   */
  [[nodiscard]] bool IsWithinMinDistanceThreshold(
      const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

  /**
   * @brief Check if velocity exceeds the activation threshold.
   * @param velocity Velocity vector to check.
   * @return True if velocity magnitude exceeds the configured threshold.
   */
  [[nodiscard]] bool ExceedsActivationVelocity(
      const Eigen::Vector2d& velocity) const;

  /**
   * @brief Project a position forward or backward in time along a velocity.
   * @param position Starting position.
   * @param velocity Velocity vector.
   * @param time_seconds Time offset (positive for future, negative for past).
   * @return Projected position.
   */
  static Eigen::Vector2d ProjectPosition(const Eigen::Vector2d& position,
                                         const Eigen::Vector2d& velocity,
                                         double time_seconds);

  /**
   * @brief Add a reading to a track at a specific position.
   * @param track_info Track to append reading to.
   * @param reading Source reading for metadata.
   * @param position Position to use for the new reading.
   */
  void AppendReadingAtPosition(TrackInfo& track_info,
                               const DynamicObstacleReading& reading,
                               const Eigen::Vector2d& position);

  /**
   * @brief Update maximum radius if candidate is larger.
   * @param current_max Current maximum radius (updated in place).
   * @param candidate Candidate radius to compare.
   */
  static void UpdateMaxRadius(float& current_max, float candidate);

  /**
   * @brief Linearly interpolate between two values.
   * @param start Starting value (t=0).
   * @param end Ending value (t=1).
   * @param t Interpolation parameter in [0, 1].
   * @return Interpolated value.
   */
  static float LinearInterpolate(float start, float end, float t);

  /**
   * @brief Append polygon frustums from readings to the frustum list.
   * @param readings Source readings containing extended polygons.
   * @param frustums Output vector to append polygon frustums to.
   */
  void AppendPolygonFrustums(
      const std::vector<DynamicObstacleReading>& readings,
      std::vector<std::unique_ptr<IClearingFrustum>>& frustums) const;

  /**
   * @brief Extract the model probability from model infos.
   * @param model the model name that the query is for
   * @param model_infos Vector of model information from obstacle detector.
   * @return Probability if model found, std::nullopt otherwise.
   */
  std::optional<double> GetModelProbability(
      const std::string& model,
      const std::vector<obstacle_detector::ModelInfo>& model_infos) const;

  /**
   * @brief Update track enabled state based on velocity and random walk
   * probability.
   *
   * Enables a track when all conditions are met:
   * - Velocity exceeds activation threshold
   * - Random walk probability is below the limit
   * - Probability has been below limit for the required duration
   *
   * Once enabled, the track stays enabled (latched).
   *
   * @param track_info Track to update.
   * @param reading Current reading with velocity and model info.
   */
  void UpdateTrackEnabledState(TrackInfo& track_info,
                               const DynamicObstacleReading& reading);

  /**
   * @brief Publish visualization markers for all tracked obstacles.
   *
   * Publishes a MarkerArray containing CYLINDER markers representing the
   * interpolated clearing circles and LINE_STRIP markers for extended polygons.
   * Enabled tracks are shown in green, disabled tracks in red.
   */
  void PublishVisualization();

  /**
   * @brief Get the visualization color for a track based on its enabled state.
   * @param enabled Whether the track is enabled.
   * @return Green for enabled tracks, red for disabled tracks.
   */
  static std_msgs::ColorRGBA GetTrackColor(bool enabled);

  /**
   * @brief Append CYLINDER markers for interpolated clearing circles.
   * @param track_info Track containing obstacle readings.
   * @param tracker_id Unique identifier for the track.
   * @param now Current timestamp for marker headers.
   * @param cutoff_time Readings older than this are skipped.
   * @param color Marker color based on track enabled state.
   * @param marker_id Running marker ID counter (incremented for each marker).
   * @param marker_array Output array to append markers to.
   */
  void AppendCircleMarkers(const TrackInfo& track_info, uint32_t tracker_id,
                           const ros::Time& now, const ros::Time& cutoff_time,
                           const std_msgs::ColorRGBA& color, int& marker_id,
                           visualization_msgs::MarkerArray& marker_array);

  /**
   * @brief Append LINE_STRIP markers for extended polygon obstacles.
   * @param track_info Track containing obstacle readings with polygons.
   * @param tracker_id Unique identifier for the track.
   * @param now Current timestamp for marker headers.
   * @param color Marker color based on track enabled state.
   * @param marker_id Running marker ID counter (incremented for each marker).
   * @param marker_array Output array to append markers to.
   */
  void AppendPolygonMarkers(const TrackInfo& track_info, uint32_t tracker_id,
                            const ros::Time& now,
                            const std_msgs::ColorRGBA& color, int& marker_id,
                            visualization_msgs::MarkerArray& marker_array);

  Config config_;
  ros::NodeHandle nh_;
  std::unordered_map<uint32_t, TrackInfo> track_map_;
  ros::Publisher visualization_pub_;
};

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_DYNAMIC_OBSTACLE_TRACKER_HPP
