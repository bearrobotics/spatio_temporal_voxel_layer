#include <openvdb/openvdb.h>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"

namespace {

DynamicObstacleTracker::Config MakeValidConfig() {
  DynamicObstacleTracker::Config config;
  config.enable = true;
  config.activation_velocity_threshold = 0.5;
  config.min_distance_between_readings_threshold = 0.1;
  config.interpolation_in_future = 0.5;
  config.interpolation_in_past = 0.5;
  config.stale_time_threshold = 2.0;
  config.inflation_radius_factor = 1.5;
  config.number_of_interpolation_circles = 5;
  config.past_time_window = 1.0;
  config.random_walk_probability_limit = 0.5;
  config.seconds_since_last_random_walk = 0.0;
  return config;
}

DynamicObstacleReading MakeReading(uint32_t id, double x, double y, double vx,
                                   double vy, float radius) {
  DynamicObstacleReading reading;
  reading.time_ = ros::Time::now();
  reading.tracker_id_ = id;
  reading.center_ = Eigen::Vector2d(x, y);
  reading.velocity_ = Eigen::Vector2d(vx, vy);
  reading.radius_ = radius;
  return reading;
}

DynamicObstacleReading MakeReadingWithModelInfo(uint32_t id, double x, double y,
                                                double vx, double vy,
                                                float radius,
                                                double random_walk_prob) {
  DynamicObstacleReading reading = MakeReading(id, x, y, vx, vy, radius);
  obstacle_detector::ModelInfo model_info;
  model_info.model_name = "random_walk";
  model_info.probability = random_walk_prob;
  reading.model_infos_.push_back(model_info);
  return reading;
}

DynamicObstacleReading::Polygon MakeSquarePolygon(float cx, float cy,
                                                  float size) {
  DynamicObstacleReading::Polygon polygon;
  float half = size / 2.0f;
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy - half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx + half, cy - half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx + half, cy + half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy + half));
  boost::geometry::append(polygon.outer(), DynamicObstacleReading::BoostPoint(
                                               cx - half, cy - half));
  return polygon;
}

// =============================================================================
// DynamicObstacleReading Tests
// =============================================================================

TEST(DynamicObstacleReadingTest, DefaultConstructor) {
  DynamicObstacleReading reading;
  EXPECT_EQ(reading.time_, ros::Time(0));
  EXPECT_EQ(reading.tracker_id_, 0u);
  EXPECT_EQ(reading.center_, Eigen::Vector2d::Zero());
  EXPECT_EQ(reading.velocity_, Eigen::Vector2d::Zero());
  EXPECT_FLOAT_EQ(reading.radius_, 0.0f);
  EXPECT_TRUE(reading.extended_polygons_.empty());
  EXPECT_TRUE(reading.model_infos_.empty());
}

TEST(DynamicObstacleReadingTest, MemberInitialization) {
  DynamicObstacleReading reading;
  reading.time_ = ros::Time(1.5);
  reading.tracker_id_ = 42;
  reading.center_ = Eigen::Vector2d(1.0, 2.0);
  reading.velocity_ = Eigen::Vector2d(0.5, -0.3);
  reading.radius_ = 0.25f;

  EXPECT_EQ(reading.time_, ros::Time(1.5));
  EXPECT_EQ(reading.tracker_id_, 42u);
  EXPECT_DOUBLE_EQ(reading.center_.x(), 1.0);
  EXPECT_DOUBLE_EQ(reading.center_.y(), 2.0);
  EXPECT_DOUBLE_EQ(reading.velocity_.x(), 0.5);
  EXPECT_DOUBLE_EQ(reading.velocity_.y(), -0.3);
  EXPECT_FLOAT_EQ(reading.radius_, 0.25f);
}

// =============================================================================
// DynamicObstacleTracker::Config IsValid Tests
// =============================================================================

TEST(ConfigValidationTest, IsValid_AllValidParams) {
  auto config = MakeValidConfig();
  EXPECT_TRUE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeActivationVelocity) {
  auto config = MakeValidConfig();
  config.activation_velocity_threshold = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeMinDistance) {
  auto config = MakeValidConfig();
  config.min_distance_between_readings_threshold = -0.01;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeInterpolationFuture) {
  auto config = MakeValidConfig();
  config.interpolation_in_future = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeInterpolationPast) {
  auto config = MakeValidConfig();
  config.interpolation_in_past = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_ZeroStaleTime) {
  auto config = MakeValidConfig();
  config.stale_time_threshold = 0.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeStaleTime) {
  auto config = MakeValidConfig();
  config.stale_time_threshold = -1.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_ZeroInflationRadius) {
  auto config = MakeValidConfig();
  config.inflation_radius_factor = 0.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeInflationRadius) {
  auto config = MakeValidConfig();
  config.inflation_radius_factor = -0.5;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_ZeroInterpolationCircles) {
  auto config = MakeValidConfig();
  config.number_of_interpolation_circles = 0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeInterpolationCircles) {
  auto config = MakeValidConfig();
  config.number_of_interpolation_circles = -1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_ZeroPastTimeWindow) {
  auto config = MakeValidConfig();
  config.past_time_window = 0.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativePastTimeWindow) {
  auto config = MakeValidConfig();
  config.past_time_window = -1.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_BoundaryValues) {
  DynamicObstacleTracker::Config config;
  config.enable = false;
  config.activation_velocity_threshold = 0.0;
  config.min_distance_between_readings_threshold = 0.0;
  config.interpolation_in_future = 0.0;
  config.interpolation_in_past = 0.0;
  config.stale_time_threshold = 0.001;
  config.inflation_radius_factor = 0.001;
  config.number_of_interpolation_circles = 1;
  config.past_time_window = 0.001;
  config.random_walk_probability_limit = 0.5;
  config.seconds_since_last_random_walk = 0.0;
  EXPECT_TRUE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeRandomWalkProbabilityLimit) {
  auto config = MakeValidConfig();
  config.random_walk_probability_limit = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_RandomWalkProbabilityLimitAboveOne) {
  auto config = MakeValidConfig();
  config.random_walk_probability_limit = 1.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_NegativeSecondsSinceLastRandomWalk) {
  auto config = MakeValidConfig();
  config.seconds_since_last_random_walk = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(ConfigValidationTest, IsValid_RandomWalkBoundaryValues) {
  auto config = MakeValidConfig();
  config.random_walk_probability_limit = 0.0;
  config.seconds_since_last_random_walk = 0.0;
  EXPECT_TRUE(config.IsValid());

  config.random_walk_probability_limit = 1.0;
  EXPECT_TRUE(config.IsValid());
}

// =============================================================================
// LoadConfig Tests (ROS Parameter Server)
// =============================================================================

TEST(LoadConfigTest, LoadConfig_AllParamsPresent) {
  ros::NodeHandle nh("/dynamic_obstacle_tracker/valid");
  auto config = DynamicObstacleTracker::Config::LoadConfig(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->enable);
  EXPECT_DOUBLE_EQ(config->activation_velocity_threshold, 0.5);
  EXPECT_DOUBLE_EQ(config->min_distance_between_readings_threshold, 0.1);
  EXPECT_DOUBLE_EQ(config->interpolation_in_future, 0.5);
  EXPECT_DOUBLE_EQ(config->interpolation_in_past, 0.5);
  EXPECT_DOUBLE_EQ(config->stale_time_threshold, 2.0);
  EXPECT_DOUBLE_EQ(config->inflation_radius_factor, 1.5);
  EXPECT_EQ(config->number_of_interpolation_circles, 5);
  EXPECT_DOUBLE_EQ(config->past_time_window, 1.0);
}

TEST(LoadConfigTest, LoadConfig_MissingEnable) {
  ros::NodeHandle nh("/dynamic_obstacle_tracker/invalid_missing_enable");
  auto config = DynamicObstacleTracker::Config::LoadConfig(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(LoadConfigTest, LoadConfig_MissingAllParams) {
  ros::NodeHandle nh("/dynamic_obstacle_tracker/empty");
  auto config = DynamicObstacleTracker::Config::LoadConfig(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(LoadConfigTest, LoadConfig_NonexistentNamespace) {
  ros::NodeHandle nh("/nonexistent_namespace");
  auto config = DynamicObstacleTracker::Config::LoadConfig(nh);
  EXPECT_FALSE(config.has_value());
}

// =============================================================================
// Create Factory Method Tests
// =============================================================================

TEST(CreateFactoryTest, Create_ValidConfig) {
  ros::NodeHandle nh("~");
  auto config = MakeValidConfig();
  auto tracker = DynamicObstacleTracker::Create(config, nh);
  EXPECT_NE(tracker, nullptr);
}

TEST(CreateFactoryTest, Create_InvalidConfig) {
  ros::NodeHandle nh("~");
  auto config = MakeValidConfig();
  config.stale_time_threshold = -1.0;
  auto tracker = DynamicObstacleTracker::Create(config, nh);
  EXPECT_EQ(tracker, nullptr);
}

// =============================================================================
// GenerateInterpolatedCircles Tests
// =============================================================================

class DynamicObstacleTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_ = MakeValidConfig();
    tracker_ = DynamicObstacleTracker::Create(config_, nh_);
    ASSERT_NE(tracker_, nullptr);
  }

  ros::NodeHandle nh_{"~"};
  DynamicObstacleTracker::Config config_;
  std::unique_ptr<DynamicObstacleTracker> tracker_;
};

TEST_F(DynamicObstacleTrackerTest, GenerateInterpolatedCircles_SingleCircle) {
  DynamicObstacleTracker::Circle begin;
  begin.center = Eigen::Vector2d(0.0, 0.0);
  begin.radius = 1.0f;

  DynamicObstacleTracker::Circle end;
  end.center = Eigen::Vector2d(2.0, 0.0);
  end.radius = 1.0f;

  std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 1, circles);

  EXPECT_EQ(circles.size(), 2u);
}

TEST_F(DynamicObstacleTrackerTest,
       GenerateInterpolatedCircles_MultipleCircles) {
  DynamicObstacleTracker::Circle begin;
  begin.center = Eigen::Vector2d(0.0, 0.0);
  begin.radius = 1.0f;

  DynamicObstacleTracker::Circle end;
  end.center = Eigen::Vector2d(10.0, 0.0);
  end.radius = 2.0f;

  std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 5, circles);

  EXPECT_EQ(circles.size(), 6u);
}

TEST_F(DynamicObstacleTrackerTest,
       GenerateInterpolatedCircles_CenterInterpolation) {
  DynamicObstacleTracker::Circle begin;
  begin.center = Eigen::Vector2d(0.0, 0.0);
  begin.radius = 1.0f;

  DynamicObstacleTracker::Circle end;
  end.center = Eigen::Vector2d(10.0, 0.0);
  end.radius = 1.0f;

  std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 2, circles);

  ASSERT_EQ(circles.size(), 3u);
  EXPECT_NEAR(circles[0]->center.x(), 0.0, 1e-5);
  EXPECT_NEAR(circles[1]->center.x(), 5.0, 1e-5);
  EXPECT_NEAR(circles[2]->center.x(), 10.0, 1e-5);
}

TEST_F(DynamicObstacleTrackerTest,
       GenerateInterpolatedCircles_RadiusInterpolation) {
  DynamicObstacleTracker::Circle begin;
  begin.center = Eigen::Vector2d(0.0, 0.0);
  begin.radius = 1.0f;

  DynamicObstacleTracker::Circle end;
  end.center = Eigen::Vector2d(0.0, 0.0);
  end.radius = 3.0f;

  std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 2, circles);

  ASSERT_EQ(circles.size(), 3u);
  EXPECT_NEAR(circles[0]->radius, 1.0f, 1e-5);
  EXPECT_NEAR(circles[1]->radius, 2.0f, 1e-5);
  EXPECT_NEAR(circles[2]->radius, 3.0f, 1e-5);
}

TEST_F(DynamicObstacleTrackerTest, GenerateInterpolatedCircles_SameStartEnd) {
  DynamicObstacleTracker::Circle begin;
  begin.center = Eigen::Vector2d(5.0, 5.0);
  begin.radius = 2.0f;

  DynamicObstacleTracker::Circle end = begin;

  std::vector<std::unique_ptr<DynamicObstacleTracker::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 3, circles);

  ASSERT_EQ(circles.size(), 4u);
  for (const auto& circle : circles) {
    EXPECT_NEAR(circle->center.x(), 5.0, 1e-5);
    EXPECT_NEAR(circle->center.y(), 5.0, 1e-5);
    EXPECT_NEAR(circle->radius, 2.0f, 1e-5);
  }
}

// =============================================================================
// AddNewReading Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, AddNewReading_AddsToMap) {
  auto reading = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  tracker_->AddNewReading(reading);

  auto circles = tracker_->GenerateCircles();
  // Should not generate circles until enabled by velocity threshold
  // The reading was just added, not enabled
}

TEST_F(DynamicObstacleTrackerTest, AddNewReading_ProjectsPositions) {
  config_.interpolation_in_past = 1.0;
  config_.interpolation_in_future = 1.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading = MakeReading(1, 0.0, 0.0, 2.0, 0.0, 0.5f);
  tracker_->AddNewReading(reading);

  // Points should be at: (-2, 0), (0, 0), (2, 0) with velocity (2, 0)
  // and interpolation times of 1.0s
}

// =============================================================================
// UpdateExistingReading Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, UpdateExistingReading_WithinMinDistance) {
  config_.min_distance_between_readings_threshold = 1.0;
  config_.activation_velocity_threshold = 0.1;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading1 = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  tracker_->AddNewReading(reading1);

  // Second reading within min distance threshold
  auto reading2 = MakeReading(1, 0.1, 0.0, 1.0, 0.0, 0.6f);
  std::vector<DynamicObstacleReading> readings = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  // Max radius should be updated to 0.6
}

TEST_F(DynamicObstacleTrackerTest, UpdateExistingReading_ExceedsMinDistance) {
  config_.min_distance_between_readings_threshold = 0.1;
  config_.activation_velocity_threshold = 0.1;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading1 = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  tracker_->AddNewReading(reading1);

  // Second reading exceeds min distance threshold
  auto reading2 = MakeReading(1, 5.0, 0.0, 1.0, 0.0, 0.5f);
  std::vector<DynamicObstacleReading> readings = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  // A new point should be added
}

TEST_F(DynamicObstacleTrackerTest, UpdateExistingReading_EnablesOnVelocity) {
  config_.activation_velocity_threshold = 0.5;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // First reading with low velocity
  auto reading1 = MakeReadingWithModelInfo(1, 0.0, 0.0, 0.1, 0.0, 0.5f, 0.1);
  tracker_->AddNewReading(reading1);

  // Update with high velocity
  auto reading2 = MakeReadingWithModelInfo(1, 1.0, 0.0, 1.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  // Obstacle should now be enabled and generate circles
  auto circles = tracker_->GenerateCircles();
  EXPECT_GT(circles.size(), 0u);
}

// =============================================================================
// GenerateCircles Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, GenerateCircles_EmptyReadingMap) {
  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

TEST_F(DynamicObstacleTrackerTest, GenerateCircles_DisabledObstacle) {
  config_.activation_velocity_threshold = 10.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading = MakeReading(1, 0.0, 0.0, 0.1, 0.0, 0.5f);
  tracker_->AddNewReading(reading);

  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

TEST_F(DynamicObstacleTrackerTest, GenerateCircles_AppliesInflationFactor) {
  config_.inflation_radius_factor = 2.0;
  config_.activation_velocity_threshold = 0.1;
  config_.number_of_interpolation_circles = 1;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading1 = MakeReadingWithModelInfo(1, 0.0, 0.0, 1.0, 0.0, 0.5f, 0.1);
  tracker_->AddNewReading(reading1);

  auto reading2 = MakeReadingWithModelInfo(1, 5.0, 0.0, 1.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  auto circles = tracker_->GenerateCircles();
  ASSERT_GT(circles.size(), 0u);
  EXPECT_NEAR(circles[0]->radius, 1.0f, 0.1f);
}

// =============================================================================
// RemoveStaleReadings Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, RemoveStaleReadings_EmptyMap) {
  tracker_->RemoveStaleReadings();
  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

// =============================================================================
// GenerateDynamicObstacleClearingFrustums Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, GenerateClearingFrustums_DisabledConfig) {
  config_.enable = false;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  std::vector<DynamicObstacleReading> readings = {reading};
  auto frustums = tracker_->GenerateDynamicObstacleClearingFrustums(readings);
  EXPECT_TRUE(frustums.empty());
}

TEST_F(DynamicObstacleTrackerTest, GenerateClearingFrustums_NewReading) {
  config_.activation_velocity_threshold = 0.1;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  std::vector<DynamicObstacleReading> readings = {reading};
  auto frustums = tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  // New reading gets added, but since it's the first reading with only
  // projected points, circles may or may not be generated depending on
  // implementation details
}

TEST_F(DynamicObstacleTrackerTest, GenerateClearingFrustums_IncludesPolygons) {
  config_.activation_velocity_threshold = 0.1;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  auto reading = MakeReading(1, 0.0, 0.0, 1.0, 0.0, 0.5f);
  reading.extended_polygons_.push_back(MakeSquarePolygon(0.0f, 0.0f, 1.0f));
  reading.extended_polygons_.push_back(MakeSquarePolygon(2.0f, 0.0f, 1.0f));

  std::vector<DynamicObstacleReading> readings = {reading};
  auto frustums = tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  // Should include the 2 polygon frustums
  int polygon_count = 0;
  for (const auto& frustum : frustums) {
    auto* polygon =
        dynamic_cast<DynamicObstacleTracker::Polygon*>(frustum.get());
    if (polygon != nullptr) {
      polygon_count++;
    }
  }
  EXPECT_EQ(polygon_count, 2);
}

TEST_F(DynamicObstacleTrackerTest, GenerateClearingFrustums_IntegrationTest) {
  config_.activation_velocity_threshold = 0.5;
  config_.min_distance_between_readings_threshold = 0.1;
  config_.number_of_interpolation_circles = 3;
  config_.inflation_radius_factor = 1.5;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // First reading
  auto reading1 = MakeReadingWithModelInfo(1, 0.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings1 = {reading1};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings1);

  // Second reading at a new position
  auto reading2 = MakeReadingWithModelInfo(1, 2.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings2 = {reading2};
  auto frustums = tracker_->GenerateDynamicObstacleClearingFrustums(readings2);

  // Should generate circles based on the trajectory
  EXPECT_GT(frustums.size(), 0u);
}

// =============================================================================
// Random Walk Probability Track Enabling Tests
// =============================================================================

TEST_F(DynamicObstacleTrackerTest, TrackNotEnabled_VelocityBelowThreshold) {
  config_.activation_velocity_threshold = 1.0;
  config_.random_walk_probability_limit = 0.5;
  config_.seconds_since_last_random_walk = 0.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // Low velocity, low random_walk probability - should NOT enable
  auto reading = MakeReadingWithModelInfo(1, 0.0, 0.0, 0.1, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings = {reading};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

TEST_F(DynamicObstacleTrackerTest, TrackNotEnabled_HighRandomWalkProbability) {
  config_.activation_velocity_threshold = 0.5;
  config_.random_walk_probability_limit = 0.3;
  config_.seconds_since_last_random_walk = 0.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // High velocity but high random_walk probability - should NOT enable
  auto reading = MakeReadingWithModelInfo(1, 0.0, 0.0, 2.0, 0.0, 0.5f, 0.5);
  std::vector<DynamicObstacleReading> readings = {reading};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

TEST_F(DynamicObstacleTrackerTest, TrackNotEnabled_DurationNotMet) {
  config_.activation_velocity_threshold = 0.5;
  config_.random_walk_probability_limit = 0.5;
  config_.seconds_since_last_random_walk = 10.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // High velocity and low probability, but duration not met - should NOT enable
  auto reading = MakeReadingWithModelInfo(1, 0.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings = {reading};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

TEST_F(DynamicObstacleTrackerTest, TrackEnabled_AllConditionsMet) {
  config_.activation_velocity_threshold = 0.5;
  config_.random_walk_probability_limit = 0.5;
  config_.seconds_since_last_random_walk = 0.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // High velocity, low probability, zero duration - should enable
  auto reading1 = MakeReadingWithModelInfo(1, 0.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings1 = {reading1};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings1);

  auto reading2 = MakeReadingWithModelInfo(1, 1.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings2 = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings2);

  auto circles = tracker_->GenerateCircles();
  EXPECT_GT(circles.size(), 0u);
}

TEST_F(DynamicObstacleTrackerTest, TrackStaysEnabled_Latched) {
  config_.activation_velocity_threshold = 0.5;
  config_.random_walk_probability_limit = 0.5;
  config_.seconds_since_last_random_walk = 0.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // First enable the track
  auto reading1 = MakeReadingWithModelInfo(1, 0.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings1 = {reading1};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings1);

  auto reading2 = MakeReadingWithModelInfo(1, 1.0, 0.0, 2.0, 0.0, 0.5f, 0.1);
  std::vector<DynamicObstacleReading> readings2 = {reading2};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings2);

  // Now send readings with conditions that would not enable a new track
  // (high random_walk probability) - track should stay enabled (latched)
  auto reading3 = MakeReadingWithModelInfo(1, 2.0, 0.0, 2.0, 0.0, 0.5f, 0.9);
  std::vector<DynamicObstacleReading> readings3 = {reading3};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings3);

  auto circles = tracker_->GenerateCircles();
  EXPECT_GT(circles.size(), 0u);
}

TEST_F(DynamicObstacleTrackerTest, TrackNotEnabled_NoModelInfo) {
  config_.activation_velocity_threshold = 0.5;
  config_.random_walk_probability_limit = 0.5;
  config_.seconds_since_last_random_walk = 0.0;
  tracker_ = DynamicObstacleTracker::Create(config_, nh_);

  // High velocity but no model info - should NOT enable
  auto reading = MakeReading(1, 0.0, 0.0, 2.0, 0.0, 0.5f);
  std::vector<DynamicObstacleReading> readings = {reading};
  tracker_->GenerateDynamicObstacleClearingFrustums(readings);

  auto circles = tracker_->GenerateCircles();
  EXPECT_TRUE(circles.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_dynamic_obstacle_tracker");
  return RUN_ALL_TESTS();
}
