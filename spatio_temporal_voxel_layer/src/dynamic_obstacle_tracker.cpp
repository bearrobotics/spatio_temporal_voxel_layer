#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"

#include "bearlib/ros/param_loader.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"

std::optional<DynamicObstacleTracker::Config>
DynamicObstacleTracker::Config::LoadConfig(ros::NodeHandle& nh) {
  using bear::lib::ros::LoadOptionalParam;

  Config config;
  auto enable = LoadOptionalParam<bool>(nh, "enable");
  auto activation_velocity_threshold =
      LoadOptionalParam<double>(nh, "activation_velocity_threshold");
  auto min_distance_between_readings_threshold =
      LoadOptionalParam<double>(nh, "min_distance_between_readings_threshold");
  auto interpolation_in_future =
      LoadOptionalParam<double>(nh, "interpolation_in_future");
  auto interpolation_in_past =
      LoadOptionalParam<double>(nh, "interpolation_in_past");
  auto stale_time_threshold =
      LoadOptionalParam<double>(nh, "stale_time_threshold");
  auto inflation_radius_factor =
      LoadOptionalParam<double>(nh, "inflation_radius_factor");
  auto number_of_interpolation_circles =
      LoadOptionalParam<int>(nh, "number_of_interpolation_circles");
  auto past_time_window = LoadOptionalParam<double>(nh, "past_time_window");
  auto random_walk_probability_limit =
      LoadOptionalParam<double>(nh, "random_walk_probability_limit");
  auto seconds_since_last_random_walk =
      LoadOptionalParam<double>(nh, "seconds_since_last_random_walk");
  auto publish_visualization =
      LoadOptionalParam<bool>(nh, "publish_visualization");
  auto max_obstacle_radius =
      LoadOptionalParam<double>(nh, "max_obstacle_radius");

  if (!enable || !activation_velocity_threshold ||
      !min_distance_between_readings_threshold || !interpolation_in_future ||
      !interpolation_in_past || !stale_time_threshold ||
      !inflation_radius_factor || !number_of_interpolation_circles ||
      !past_time_window || !random_walk_probability_limit ||
      !seconds_since_last_random_walk || !publish_visualization ||
      !max_obstacle_radius) {
    return std::nullopt;
  }

  config.enable = *enable;
  config.activation_velocity_threshold = *activation_velocity_threshold;
  config.min_distance_between_readings_threshold =
      *min_distance_between_readings_threshold;
  config.interpolation_in_future = *interpolation_in_future;
  config.interpolation_in_past = *interpolation_in_past;
  config.stale_time_threshold = *stale_time_threshold;
  config.inflation_radius_factor = *inflation_radius_factor;
  config.number_of_interpolation_circles = *number_of_interpolation_circles;
  config.past_time_window = *past_time_window;
  config.random_walk_probability_limit = *random_walk_probability_limit;
  config.seconds_since_last_random_walk = *seconds_since_last_random_walk;
  config.publish_visualization = *publish_visualization;
  config.max_obstacle_radius = *max_obstacle_radius;

  return config;
}

bool DynamicObstacleTracker::Config::IsValid() const {
  if (activation_velocity_threshold < 0) {
    ROS_ERROR("activation_velocity_threshold must be >= 0");
    return false;
  }
  if (min_distance_between_readings_threshold < 0) {
    ROS_ERROR("min_distance_between_readings_threshold must be >= 0");
    return false;
  }
  if (interpolation_in_future < 0) {
    ROS_ERROR("interpolation_in_future must be >= 0");
    return false;
  }
  if (interpolation_in_past < 0) {
    ROS_ERROR("interpolation_in_past must be >= 0");
    return false;
  }
  if (stale_time_threshold <= 0) {
    ROS_ERROR("stale_time_threshold must be > 0");
    return false;
  }
  if (inflation_radius_factor <= 0) {
    ROS_ERROR("inflation_radius_factor must be > 0");
    return false;
  }
  if (number_of_interpolation_circles < 1) {
    ROS_ERROR("number_of_interpolation_circles must be >= 1");
    return false;
  }
  if (past_time_window <= 0) {
    ROS_ERROR("past_time_window must be > 0");
    return false;
  }
  if (random_walk_probability_limit < 0 || random_walk_probability_limit > 1) {
    ROS_ERROR("random_walk_probability_limit must be between 0 and 1");
    return false;
  }
  if (seconds_since_last_random_walk < 0) {
    ROS_ERROR("seconds_since_last_random_walk must be >= 0");
    return false;
  }
  if (max_obstacle_radius <= 0) {
    ROS_ERROR("max_obstacle_radius must be > 0");
    return false;
  }
  return true;
}

std::unique_ptr<DynamicObstacleTracker> DynamicObstacleTracker::Create(
    const Config& config, ros::NodeHandle& nh) {
  if (!config.IsValid()) {
    return nullptr;
  }
  return std::unique_ptr<DynamicObstacleTracker>(
      new DynamicObstacleTracker(config, nh));
}

DynamicObstacleTracker::DynamicObstacleTracker(const Config& config,
                                               ros::NodeHandle& nh)
    : config_(config), nh_(nh) {
  if (config_.publish_visualization) {
    visualization_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("visualization", 1);
  }
}

double DynamicObstacleTracker::SquaredDistance(const Eigen::Vector2d& a,
                                               const Eigen::Vector2d& b) {
  return (a - b).squaredNorm();
}

bool DynamicObstacleTracker::IsWithinMinDistanceThreshold(
    const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  const double threshold_squared =
      config_.min_distance_between_readings_threshold *
      config_.min_distance_between_readings_threshold;
  return SquaredDistance(a, b) < threshold_squared;
}

bool DynamicObstacleTracker::ExceedsActivationVelocity(
    const Eigen::Vector2d& velocity) const {
  return velocity.norm() > config_.activation_velocity_threshold;
}

Eigen::Vector2d DynamicObstacleTracker::ProjectPosition(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    double time_seconds) {
  return position + (velocity * time_seconds);
}

void DynamicObstacleTracker::UpdateMaxRadius(float& current_max,
                                             float candidate) {
  if (candidate > current_max) {
    current_max = candidate;
  }
}

float DynamicObstacleTracker::LinearInterpolate(float start, float end,
                                                float t) {
  return start * (1.0f - t) + end * t;
}

void DynamicObstacleTracker::GenerateInterpolatedCircles(
    Circle begin_circle, Circle end_circle, int num_circles,
    std::vector<std::unique_ptr<Circle>>& interpolated_circles) const {
  for (int i = 0; i <= num_circles; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(num_circles);
    auto circle = std::make_unique<Circle>();
    circle->center.x() =
        LinearInterpolate(begin_circle.center.x(), end_circle.center.x(), t);
    circle->center.y() =
        LinearInterpolate(begin_circle.center.y(), end_circle.center.y(), t);
    circle->radius =
        LinearInterpolate(begin_circle.radius, end_circle.radius, t);
    interpolated_circles.push_back(std::move(circle));
  }
}

void DynamicObstacleTracker::UpdateExistingReading(
    std::unordered_map<uint32_t, TrackInfo>::iterator& obstacle_entry,
    const DynamicObstacleReading& reading) {
  TrackInfo& track_info = obstacle_entry->second;
  DynamicObstacleReading& last_reading = track_info.readings.back();

  if (IsWithinMinDistanceThreshold(reading.center_, last_reading.center_)) {
    last_reading.time_ = reading.time_;
    UpdateMaxRadius(last_reading.radius_, reading.radius_);
    UpdateMaxRadius(track_info.max_radius, reading.radius_);
    UpdateTrackEnabledState(track_info, reading);
    return;
  }

  track_info.readings.emplace_back(std::move(reading));
  UpdateMaxRadius(track_info.max_radius, reading.radius_);
  UpdateTrackEnabledState(track_info, reading);
}

void DynamicObstacleTracker::AppendReadingAtPosition(
    TrackInfo& track_info, const DynamicObstacleReading& reading,
    const Eigen::Vector2d& position) {
  DynamicObstacleReading& projected_reading =
      track_info.readings.emplace_back();
  projected_reading.time_ = reading.time_;
  projected_reading.tracker_id_ = reading.tracker_id_;
  projected_reading.center_ = position;
  projected_reading.velocity_ = reading.velocity_;
  projected_reading.radius_ = reading.radius_;
}

void DynamicObstacleTracker::AddNewReading(
    const DynamicObstacleReading& reading) {
  TrackInfo& track_info =
      track_map_.emplace(reading.tracker_id_, TrackInfo()).first->second;

  track_info.last_exceeded_random_walk_limit = ros::Time::now();

  Eigen::Vector2d past_position = ProjectPosition(
      reading.center_, reading.velocity_, -config_.interpolation_in_past);
  AppendReadingAtPosition(track_info, reading, past_position);

  track_info.readings.push_back(reading);

  Eigen::Vector2d future_position = ProjectPosition(
      reading.center_, reading.velocity_, config_.interpolation_in_future);
  AppendReadingAtPosition(track_info, reading, future_position);

  track_info.max_radius = reading.radius_;
  UpdateTrackEnabledState(track_info, reading);
}

std::vector<std::unique_ptr<geometry::IClearingFrustum>>
DynamicObstacleTracker::GenerateDynamicObstacleClearingFrustums(
    std::vector<DynamicObstacleReading>& readings) {
  if (!config_.enable) {
    return {};
  }
  for (const auto& reading : readings) {
    auto it = track_map_.find(reading.tracker_id_);
    if (it == track_map_.end()) {
      AddNewReading(reading);
      auto new_it = track_map_.find(reading.tracker_id_);
      if (new_it->second.max_radius > config_.max_obstacle_radius) {
        track_map_.erase(new_it);
      }
      continue;
    }
    UpdateExistingReading(it, reading);
    if (it->second.max_radius > config_.max_obstacle_radius) {
      track_map_.erase(it);
    }
  }
  RemoveStaleReadings();

  std::vector<std::unique_ptr<IClearingFrustum>> frustums;
  for (auto& circle : GenerateCircles()) {
    frustums.push_back(std::move(circle));
  }
  AppendPolygonFrustums(readings, frustums);

  if (config_.publish_visualization) {
    PublishVisualization();
  }

  return frustums;
}

void DynamicObstacleTracker::AppendPolygonFrustums(
    const std::vector<DynamicObstacleReading>& readings,
    std::vector<std::unique_ptr<IClearingFrustum>>& frustums) const {
  for (const DynamicObstacleReading& reading : readings) {
    for (const auto& polygon : reading.extended_polygons_) {
      auto polygon_frustum = std::make_unique<Polygon>();
      polygon_frustum->polygon_ = polygon;
      frustums.push_back(std::move(polygon_frustum));
    }
  }
}

std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>>
DynamicObstacleTracker::GenerateCircles() {
  std::vector<std::unique_ptr<Circle>> circles;
  const ros::Time cutoff_time =
      ros::Time::now() - ros::Duration(config_.past_time_window);

  for (const auto& [tracker_id, track_info] : track_map_) {
    if (!track_info.enabled) {
      continue;
    }

    Circle previous_circle;
    bool has_previous_point = false;

    for (const DynamicObstacleReading& reading : track_info.readings) {
      if (reading.time_ < cutoff_time) {
        continue;
      }

      Circle current_circle;
      current_circle.center = reading.center_;
      current_circle.radius =
          track_info.max_radius * config_.inflation_radius_factor;

      if (has_previous_point) {
        GenerateInterpolatedCircles(previous_circle, current_circle,
                                    config_.number_of_interpolation_circles,
                                    circles);
      }

      previous_circle = current_circle;
      has_previous_point = true;
    }
  }
  return circles;
}

void DynamicObstacleTracker::RemoveStaleReadings() {
  for (auto it = track_map_.begin(); it != track_map_.end();) {
    const double time_diff =
        (ros::Time::now() - it->second.readings.back().time_).toSec();
    if (time_diff > config_.stale_time_threshold) {
      it = track_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<double> DynamicObstacleTracker::GetModelProbability(
    const std::string& model,
    const std::vector<obstacle_detector::ModelInfo>& model_infos) const {
  ROS_DEBUG_THROTTLE(1.0, "Getting model probability for model '%s'",
                     model.c_str());
  for (const auto& model_info : model_infos) {
    ROS_DEBUG_THROTTLE(1.0, "Model info: %s with probability %.3f",
                       model_info.model_name.c_str(), model_info.probability);
    if (model_info.model_name == model) {
      return model_info.probability;
    }
  }
  return std::nullopt;
}

void DynamicObstacleTracker::UpdateTrackEnabledState(
    TrackInfo& track_info, const DynamicObstacleReading& reading) {
  const uint32_t id = reading.tracker_id_;

  if (track_info.enabled) {
    ROS_DEBUG_THROTTLE(1.0, "[Track %u] Already enabled, skipping", id);
    return;
  }

  const double velocity_magnitude = reading.velocity_.norm();
  if (!ExceedsActivationVelocity(reading.velocity_)) {
    ROS_DEBUG_THROTTLE(1.0,
                       "[Track %u] Velocity %.2f m/s below threshold %.2f m/s, "
                       "not enabling",
                       id, velocity_magnitude,
                       config_.activation_velocity_threshold);
    return;
  }

  auto random_walk_probability =
      GetModelProbability("random_walk", reading.model_infos_);
  if (!random_walk_probability) {
    ROS_DEBUG_THROTTLE(1.0, "[Track %u] No random_walk model info available",
                       id);
    return;
  }

  if (*random_walk_probability >= config_.random_walk_probability_limit) {
    track_info.last_exceeded_random_walk_limit = ros::Time::now();
    ROS_DEBUG_THROTTLE(1.0,
                       "[Track %u] Random walk probability %.3f >= limit %.3f, "
                       "resetting timer",
                       id, *random_walk_probability,
                       config_.random_walk_probability_limit);
    return;
  }

  const double time_since_exceeded =
      (ros::Time::now() - track_info.last_exceeded_random_walk_limit).toSec();
  if (time_since_exceeded >= config_.seconds_since_last_random_walk) {
    track_info.enabled = true;
    ROS_DEBUG_THROTTLE(
        1.0,
        "[Track %u] Enabled: velocity=%.2f m/s, random_walk_prob=%.3f, "
        "time_since_exceeded=%.1fs",
        id, velocity_magnitude, *random_walk_probability, time_since_exceeded);
  } else {
    ROS_DEBUG_THROTTLE(
        1.0,
        "[Track %u] Time since last random walk %.1fs < required %.1fs, "
        "not yet enabling",
        id, time_since_exceeded, config_.seconds_since_last_random_walk);
  }
}

std_msgs::ColorRGBA DynamicObstacleTracker::GetTrackColor(bool enabled) {
  std_msgs::ColorRGBA color;
  color.a = 0.25f;
  color.r = enabled ? 0.0f : 1.0f;
  color.g = enabled ? 1.0f : 0.0f;
  color.b = 0.0f;
  return color;
}

void DynamicObstacleTracker::AppendCircleMarkers(
    const TrackInfo& track_info, uint32_t tracker_id, const ros::Time& now,
    const ros::Time& cutoff_time, const std_msgs::ColorRGBA& color,
    int& marker_id, visualization_msgs::MarkerArray& marker_array) {
  const std::string marker_namespace = "track_" + std::to_string(tracker_id);
  const float inflated_radius =
      track_info.max_radius * config_.inflation_radius_factor;
  constexpr float kCylinderHeight = 0.01f;

  Circle previous_circle;
  bool has_previous_reading = false;

  for (const DynamicObstacleReading& reading : track_info.readings) {
    if (reading.time_ < cutoff_time) {
      continue;
    }

    Circle current_circle;
    current_circle.center = reading.center_;
    current_circle.radius = inflated_radius;

    if (!has_previous_reading) {
      previous_circle = current_circle;
      has_previous_reading = true;
      continue;
    }

    std::vector<std::unique_ptr<Circle>> interpolated_circles;
    GenerateInterpolatedCircles(previous_circle, current_circle,
                                config_.number_of_interpolation_circles,
                                interpolated_circles);

    for (const auto& circle : interpolated_circles) {
      visualization_msgs::Marker& marker = marker_array.markers.emplace_back();
      marker.header.frame_id = "map";
      marker.header.stamp = now;
      marker.ns = marker_namespace;
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = circle->center.x();
      marker.pose.position.y = circle->center.y();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = circle->radius * 2.0;
      marker.scale.y = circle->radius * 2.0;
      marker.scale.z = kCylinderHeight;
      marker.color = color;
    }

    previous_circle = current_circle;
  }
}

void DynamicObstacleTracker::AppendPolygonMarkers(
    const TrackInfo& track_info, uint32_t tracker_id, const ros::Time& now,
    const std_msgs::ColorRGBA& color, int& marker_id,
    visualization_msgs::MarkerArray& marker_array) {
  const std::string marker_namespace = "polygon_" + std::to_string(tracker_id);
  constexpr float kLineWidth = 0.02f;

  for (const DynamicObstacleReading& reading : track_info.readings) {
    for (const auto& polygon : reading.extended_polygons_) {
      visualization_msgs::Marker& marker = marker_array.markers.emplace_back();
      marker.header.frame_id = "map";
      marker.header.stamp = now;
      marker.ns = marker_namespace;
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = kLineWidth;
      marker.color = color;

      const auto& outer_ring = polygon.outer();
      for (const auto& point : outer_ring) {
        geometry_msgs::Point& p = marker.points.emplace_back();
        p.x = boost::geometry::get<0>(point);
        p.y = boost::geometry::get<1>(point);
        p.z = 0.0;
      }

      if (outer_ring.empty()) {
        continue;
      }

      geometry_msgs::Point& closing_point = marker.points.emplace_back();
      closing_point.x = boost::geometry::get<0>(outer_ring.front());
      closing_point.y = boost::geometry::get<1>(outer_ring.front());
      closing_point.z = 0.0;
    }
  }
}

void DynamicObstacleTracker::PublishVisualization() {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker& delete_all_marker =
      marker_array.markers.emplace_back();
  delete_all_marker.action = visualization_msgs::Marker::DELETEALL;

  const ros::Time now = ros::Time::now();
  const ros::Time cutoff_time = now - ros::Duration(config_.past_time_window);
  int marker_id = 0;

  for (const auto& [tracker_id, track_info] : track_map_) {
    if (!track_info.enabled) {
      continue;
    }
    const std_msgs::ColorRGBA color = GetTrackColor(track_info.enabled);
    AppendCircleMarkers(track_info, tracker_id, now, cutoff_time, color,
                        marker_id, marker_array);
    AppendPolygonMarkers(track_info, tracker_id, now, color, marker_id,
                         marker_array);
  }

  visualization_pub_.publish(marker_array);
}
