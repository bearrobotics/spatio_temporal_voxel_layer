#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_TRACKER_HPP
#define SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_TRACKER_HPP

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_models/circle_frustum.h"
#include "spatio_temporal_voxel_layer/frustum_models/clearing_frustum.h"
#include "spatio_temporal_voxel_layer/frustum_models/polygon_frustum.h"
#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"
#include "visualization_msgs/MarkerArray.h"

/**
 * @class RobotMotionTracker
 * @brief Tracks other robots via RobotMotion messages and generates clearing
 *        frustums for the spatio-temporal voxel layer.
 *
 * Maintains a per-robot history of positions keyed by robot_id. Once a robot's
 * velocity exceeds the configured threshold, interpolated circles and footprint
 * polygons are generated along its trajectory to clear stale voxels.
 */
class RobotMotionTracker {
 public:
  using Circle = geometry::Circle;
  using IClearingFrustum = geometry::IClearingFrustum;

  /**
   * @brief Configuration parameters for the robot motion tracker.
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
     * @return True if all parameters are set to values that are considered
     * valid, false otherwise.
     */
    bool IsValid() const;

    /// Whether robot motion tracking is enabled.
    bool enable = false;
    /// Minimum velocity (m/s) for a robot to generate clearing circles.
    double activation_velocity_threshold = -1.0;
    /// Minimum distance (m) between readings to add a new point.
    double min_distance_between_readings_threshold = -1.0;
    /// Time (s) after which a track is removed if no new message is received.
    double stale_time_threshold = -1.0;
    /// Factor to multiply robot_radius for clearing circle inflation.
    double inflation_radius_factor = -1.0;
    /// Number of circles to interpolate between consecutive readings.
    int number_of_interpolation_circles = -1;
    /// Time window (s) for considering past readings when generating circles.
    double past_time_window = -1.0;
    /// Whether to publish visualization markers.
    bool publish_visualization = false;
  };

  /**
   * @brief Stores the tracking history for a single robot.
   */
  struct TrackInfo {
    /// Whether this robot has exceeded the velocity threshold (latched on).
    bool enabled = false;
    /// History of tracked readings for this robot.
    std::vector<RobotMotionReading> readings;
    /// Maximum observed radius for this robot.
    float max_radius = 0.0f;
  };

  /**
   * @brief Factory function to create a RobotMotionTracker instance.
   * @param config Configuration parameters.
   * @param nh ROS NodeHandle for the tracker.
   * @return Unique pointer to the tracker, or nullptr if config is invalid.
   */
  static std::unique_ptr<RobotMotionTracker> Create(const Config& config,
                                                    ros::NodeHandle& nh);

  /**
   * @brief Generate interpolated circles between two positions.
   * @param begin_circle Starting circle position and radius.
   * @param end_circle Ending circle position and radius.
   * @param num_circles Number of circles to interpolate.
   * @param interpolated_circles Output vector to append generated circles.
   */
  void GenerateInterpolatedCircles(
      Circle begin_circle, Circle end_circle, int num_circles,
      std::vector<std::unique_ptr<Circle>>& interpolated_circles) const;

  /**
   * @brief Update tracking data for an existing robot.
   * @param track_info Track info for the robot to update.
   * @param reading New reading data for the robot.
   */
  void UpdateExistingReading(TrackInfo& track_info,
                             const RobotMotionReading& reading);

  /**
   * @brief Add a new robot to the tracking map.
   * @param reading Initial reading data for the new robot.
   */
  void AddNewReading(const RobotMotionReading& reading);

  /**
   * @brief Process robot motion readings and generate clearing frustums.
   *
   * This is the main entry point for the tracker. It updates internal tracking
   * state, removes stale tracks, and generates clearing frustums (circles and
   * footprint polygons) for use by the voxel layer.
   *
   * @param readings Vector of robot motion readings to process.
   * @return Vector of clearing frustums covering robot trajectories.
   */
  std::vector<std::unique_ptr<IClearingFrustum>>
  GenerateRobotMotionClearingFrustums(
      std::vector<RobotMotionReading>& readings);

  /**
   * @brief Remove robots that haven't been updated within the stale threshold.
   */
  void RemoveStaleReadings();

 private:
  /**
   * @brief Generate clearing circles from all tracked robots.
   * @return Vector of circles covering tracked robot trajectories.
   */
  std::vector<std::unique_ptr<Circle>> GenerateCircles();
  /**
   * @brief Construct a RobotMotionTracker with the given configuration.
   * @param config Configuration parameters.
   * @param nh ROS NodeHandle for the tracker.
   */
  explicit RobotMotionTracker(const Config& config, ros::NodeHandle& nh);

  /**
   * @brief Compute the squared Euclidean distance between two 2D points.
   */
  static double SquaredDistance(const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b);

  /**
   * @brief Check if two points are within the minimum distance threshold.
   */
  [[nodiscard]] bool IsWithinMinDistanceThreshold(
      const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

  /**
   * @brief Check if velocity exceeds the activation threshold.
   */
  [[nodiscard]] bool ExceedsActivationVelocity(
      const Eigen::Vector2d& velocity) const;

  /**
   * @brief Update maximum radius if candidate is larger.
   */
  static void UpdateMaxRadius(float& current_max, float candidate);

  /**
   * @brief Linearly interpolate between two values.
   */
  static float LinearInterpolate(float start, float end, float t);

  /**
   * @brief Update track enabled state based on velocity threshold.
   *
   * Enables a track when velocity exceeds the activation threshold.
   * Once enabled, the track stays enabled (latched).
   *
   * @param track_info Track to update.
   * @param reading Current reading with velocity.
   */
  void UpdateTrackEnabledState(TrackInfo& track_info,
                               const RobotMotionReading& reading);

  /**
   * @brief Append polygon frustums from the footprint of each reading.
   * @param readings Source readings containing footprint polygons.
   * @param frustums Output vector to append polygon frustums to.
   */
  void AppendFootprintFrustums(
      const std::vector<RobotMotionReading>& readings,
      std::vector<std::unique_ptr<IClearingFrustum>>& frustums) const;

  /**
   * @brief Publish visualization markers for all tracked robots.
   */
  void PublishVisualization();

  /**
   * @brief Append CYLINDER markers for interpolated clearing circles.
   */
  void AppendCircleMarkers(const TrackInfo& track_info,
                           const std::string& robot_id, const ros::Time& now,
                           const ros::Time& cutoff_time,
                           const std_msgs::ColorRGBA& color, int& marker_id,
                           visualization_msgs::MarkerArray& marker_array);

  /**
   * @brief Append LINE_STRIP markers for footprint polygons.
   */
  void AppendFootprintMarkers(const TrackInfo& track_info,
                              const std::string& robot_id, const ros::Time& now,
                              const std_msgs::ColorRGBA& color, int& marker_id,
                              visualization_msgs::MarkerArray& marker_array);

  /**
   * @brief Get the visualization color for a track based on its enabled state.
   * @param enabled Whether the track is enabled.
   * @return Green for enabled tracks, red for disabled tracks.
   */
  static std_msgs::ColorRGBA GetTrackColor(bool enabled);

  Config config_;
  ros::NodeHandle nh_;
  std::unordered_map<std::string, TrackInfo> track_map_;
  ros::Publisher visualization_pub_;
};

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_ROBOT_MOTION_TRACKER_HPP
