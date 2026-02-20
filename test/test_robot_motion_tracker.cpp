#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/robot_motion_reading.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"

namespace {

RobotMotionTracker::Config MakeValidConfig() {
  RobotMotionTracker::Config config;
  config.enable = true;
  config.activation_velocity_threshold = 0.1;
  config.min_distance_between_readings_threshold = 0.25;
  config.stale_time_threshold = 2.0;
  config.inflation_radius_factor = 1.2;
  config.number_of_interpolation_circles = 3;
  config.past_time_window = 1.0;
  config.publish_visualization = false;
  return config;
}

RobotMotionReading MakeReading(const std::string& robot_id, double x, double y,
                               double vx, double vy, float radius) {
  RobotMotionReading reading;
  reading.time_ = ros::Time::now();
  reading.robot_id_ = robot_id;
  reading.center_ = Eigen::Vector2d(x, y);
  reading.velocity_ = Eigen::Vector2d(vx, vy);
  reading.radius_ = radius;
  return reading;
}

RobotMotionReading MakeReadingWithFootprint(const std::string& robot_id,
                                            double x, double y, double vx,
                                            double vy, float radius,
                                            float footprint_size) {
  RobotMotionReading reading = MakeReading(robot_id, x, y, vx, vy, radius);
  float half = footprint_size / 2.0f;
  boost::geometry::append(reading.footprint_.outer(),
                          RobotMotionReading::BoostPoint(x - half, y - half));
  boost::geometry::append(reading.footprint_.outer(),
                          RobotMotionReading::BoostPoint(x + half, y - half));
  boost::geometry::append(reading.footprint_.outer(),
                          RobotMotionReading::BoostPoint(x + half, y + half));
  boost::geometry::append(reading.footprint_.outer(),
                          RobotMotionReading::BoostPoint(x - half, y + half));
  boost::geometry::append(reading.footprint_.outer(),
                          RobotMotionReading::BoostPoint(x - half, y - half));
  return reading;
}

// =============================================================================
// RobotMotionReading Tests
// =============================================================================

TEST(RobotMotionReadingTest, DefaultConstructor) {
  RobotMotionReading reading;
  EXPECT_EQ(reading.time_, ros::Time(0));
  EXPECT_TRUE(reading.robot_id_.empty());
  EXPECT_EQ(reading.center_, Eigen::Vector2d::Zero());
  EXPECT_EQ(reading.velocity_, Eigen::Vector2d::Zero());
  EXPECT_FLOAT_EQ(reading.radius_, 0.0f);
  EXPECT_TRUE(reading.footprint_.outer().empty());
}

TEST(RobotMotionReadingTest, MemberInitialization) {
  RobotMotionReading reading;
  reading.time_ = ros::Time(1.5);
  reading.robot_id_ = "robot_1";
  reading.center_ = Eigen::Vector2d(1.0, 2.0);
  reading.velocity_ = Eigen::Vector2d(0.5, -0.3);
  reading.radius_ = 0.3f;

  EXPECT_EQ(reading.time_, ros::Time(1.5));
  EXPECT_EQ(reading.robot_id_, "robot_1");
  EXPECT_DOUBLE_EQ(reading.center_.x(), 1.0);
  EXPECT_DOUBLE_EQ(reading.center_.y(), 2.0);
  EXPECT_DOUBLE_EQ(reading.velocity_.x(), 0.5);
  EXPECT_DOUBLE_EQ(reading.velocity_.y(), -0.3);
  EXPECT_FLOAT_EQ(reading.radius_, 0.3f);
}

// =============================================================================
// RobotMotionTracker::Config IsValid Tests
// =============================================================================

TEST(RobotMotionTrackerConfigTest, ValidConfig) {
  EXPECT_TRUE(MakeValidConfig().IsValid());
}

TEST(RobotMotionTrackerConfigTest, NegativeActivationVelocityThreshold) {
  auto config = MakeValidConfig();
  config.activation_velocity_threshold = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(RobotMotionTrackerConfigTest, NegativeMinDistance) {
  auto config = MakeValidConfig();
  config.min_distance_between_readings_threshold = -0.1;
  EXPECT_FALSE(config.IsValid());
}

TEST(RobotMotionTrackerConfigTest, ZeroStaleTimeThreshold) {
  auto config = MakeValidConfig();
  config.stale_time_threshold = 0.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(RobotMotionTrackerConfigTest, ZeroInflationRadiusFactor) {
  auto config = MakeValidConfig();
  config.inflation_radius_factor = 0.0;
  EXPECT_FALSE(config.IsValid());
}

TEST(RobotMotionTrackerConfigTest, ZeroInterpolationCircles) {
  auto config = MakeValidConfig();
  config.number_of_interpolation_circles = 0;
  EXPECT_FALSE(config.IsValid());
}

TEST(RobotMotionTrackerConfigTest, ZeroPastTimeWindow) {
  auto config = MakeValidConfig();
  config.past_time_window = 0.0;
  EXPECT_FALSE(config.IsValid());
}

// =============================================================================
// RobotMotionTracker::Config LoadConfig Tests
// =============================================================================

TEST(RobotMotionTrackerConfigTest, LoadValidConfig) {
  ros::NodeHandle nh("/robot_motion_tracker/valid");
  auto config = RobotMotionTracker::Config::LoadConfig(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->enable);
  EXPECT_DOUBLE_EQ(config->activation_velocity_threshold, 0.1);
  EXPECT_DOUBLE_EQ(config->min_distance_between_readings_threshold, 0.25);
  EXPECT_DOUBLE_EQ(config->stale_time_threshold, 2.0);
  EXPECT_DOUBLE_EQ(config->inflation_radius_factor, 1.2);
  EXPECT_EQ(config->number_of_interpolation_circles, 3);
  EXPECT_DOUBLE_EQ(config->past_time_window, 1.0);
  EXPECT_FALSE(config->publish_visualization);
}

TEST(RobotMotionTrackerConfigTest, LoadMissingEnableReturnsNullopt) {
  ros::NodeHandle nh("/robot_motion_tracker/invalid_missing_enable");
  auto config = RobotMotionTracker::Config::LoadConfig(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(RobotMotionTrackerConfigTest, LoadEmptyConfigReturnsNullopt) {
  ros::NodeHandle nh("/robot_motion_tracker/empty");
  auto config = RobotMotionTracker::Config::LoadConfig(nh);
  EXPECT_FALSE(config.has_value());
}

// =============================================================================
// RobotMotionTracker Tests
// =============================================================================

class RobotMotionTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle nh;
    tracker_ = RobotMotionTracker::Create(MakeValidConfig(), nh);
    ASSERT_NE(tracker_, nullptr);
  }

  std::unique_ptr<RobotMotionTracker> tracker_;
};

TEST_F(RobotMotionTrackerTest, DisabledTrackerReturnsNoFrustums) {
  RobotMotionTracker::Config config = MakeValidConfig();
  config.enable = false;
  ros::NodeHandle nh;
  auto tracker = RobotMotionTracker::Create(config, nh);

  std::vector<RobotMotionReading> readings = {
      MakeReading("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f)};
  auto frustums = tracker->GenerateRobotMotionClearingFrustums(readings);
  EXPECT_TRUE(frustums.empty());
}

TEST_F(RobotMotionTrackerTest, SingleReadingProducesNoCircles) {
  // A single reading has no previous point to interpolate from, so no circles.
  // It does produce a footprint polygon frustum if footprint is set.
  std::vector<RobotMotionReading> readings = {
      MakeReading("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings);
  EXPECT_TRUE(frustums.empty());
}

TEST_F(RobotMotionTrackerTest,
       SingleReadingWithFootprintProducesPolygonFrustum) {
  std::vector<RobotMotionReading> readings = {
      MakeReadingWithFootprint("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f, 0.6f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings);
  EXPECT_EQ(frustums.size(), 1u);
}

TEST_F(RobotMotionTrackerTest, BelowVelocityThresholdNoCircles) {
  // First call — adds track but doesn't enable it (velocity too low).
  std::vector<RobotMotionReading> readings1 = {
      MakeReading("robot_1", 0.0, 0.0, 0.0, 0.0, 0.3f)};
  tracker_->GenerateRobotMotionClearingFrustums(readings1);

  // Second call — second position, still below threshold.
  std::vector<RobotMotionReading> readings2 = {
      MakeReading("robot_1", 1.0, 0.0, 0.0, 0.0, 0.3f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings2);
  EXPECT_TRUE(frustums.empty());
}

TEST_F(RobotMotionTrackerTest, AboveVelocityThresholdProducesCircles) {
  // First call — adds track with velocity above threshold (enables it).
  std::vector<RobotMotionReading> readings1 = {
      MakeReading("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f)};
  tracker_->GenerateRobotMotionClearingFrustums(readings1);

  // Second call — second position, far enough apart to add a new reading.
  std::vector<RobotMotionReading> readings2 = {
      MakeReading("robot_1", 1.0, 0.0, 1.0, 0.0, 0.3f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings2);
  EXPECT_GT(frustums.size(), 0u);
}

TEST_F(RobotMotionTrackerTest, DifferentRobotIdsProduceSeparateTracks) {
  // Two different robots, both moving fast enough to enable tracks.
  std::vector<RobotMotionReading> readings1 = {
      MakeReading("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f),
      MakeReading("robot_2", 5.0, 5.0, 0.0, 1.0, 0.3f)};
  tracker_->GenerateRobotMotionClearingFrustums(readings1);

  std::vector<RobotMotionReading> readings2 = {
      MakeReading("robot_1", 1.0, 0.0, 1.0, 0.0, 0.3f),
      MakeReading("robot_2", 5.0, 6.0, 0.0, 1.0, 0.3f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings2);
  // Both tracks should produce circles; expect more than from a single track.
  EXPECT_GT(frustums.size(), 0u);
}

TEST_F(RobotMotionTrackerTest, StaleTrackDoesNotProduceCirclesAfterReset) {
  // Verify that a track evicted by staleness and re-added as a single fresh
  // reading produces no circles (it needs at least 2 readings for
  // interpolation).
  //
  // Strategy: build up a 2-reading track that produces circles, then feed a
  // reading whose timestamp is explicitly in the past to make the track stale,
  // then call again with a fresh reading to trigger eviction and re-add.

  RobotMotionTracker::Config config = MakeValidConfig();
  config.stale_time_threshold = 0.1;  // 100ms
  config.past_time_window = 10.0;     // wide window so old readings show up
  ros::NodeHandle nh;
  auto tracker = RobotMotionTracker::Create(config, nh);

  // Two readings with current timestamps, far enough apart → circles.
  ros::Time t_now = ros::Time::now();
  RobotMotionReading r1 = MakeReading("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f);
  r1.time_ = t_now;
  RobotMotionReading r2 = MakeReading("robot_1", 1.0, 0.0, 1.0, 0.0, 0.3f);
  r2.time_ = t_now;

  std::vector<RobotMotionReading> readings1 = {r1};
  tracker->GenerateRobotMotionClearingFrustums(readings1);
  std::vector<RobotMotionReading> readings2 = {r2};
  auto before_frustums =
      tracker->GenerateRobotMotionClearingFrustums(readings2);
  EXPECT_GT(before_frustums.size(), 0u);

  // Feed a stale-timestamped reading to make the track's last reading old.
  // Then RemoveStaleReadings (called inside the next GenerateRobotMotion call)
  // will evict it because the last reading is > stale_time_threshold old.
  RobotMotionReading stale = MakeReading("robot_1", 1.5, 0.0, 1.0, 0.0, 0.3f);
  stale.time_ = ros::Time(0.001);  // epoch + 1ms = effectively in the past
  std::vector<RobotMotionReading> stale_readings = {stale};
  tracker->GenerateRobotMotionClearingFrustums(stale_readings);

  // Now call with a fresh reading — stale track is evicted, fresh one has 1
  // reading only → no circle pairs, no footprint polygon.
  std::vector<RobotMotionReading> fresh_readings = {
      MakeReading("robot_1", 2.0, 0.0, 1.0, 0.0, 0.3f)};
  auto after_frustums =
      tracker->GenerateRobotMotionClearingFrustums(fresh_readings);
  EXPECT_TRUE(after_frustums.empty());
}

TEST_F(RobotMotionTrackerTest, FootprintFrustumContainsPointInsideFootprint) {
  std::vector<RobotMotionReading> readings = {
      MakeReadingWithFootprint("robot_1", 0.0, 0.0, 1.0, 0.0, 0.3f, 1.0f)};
  auto frustums = tracker_->GenerateRobotMotionClearingFrustums(readings);
  ASSERT_EQ(frustums.size(), 1u);

  // The footprint is a 1x1 square centered at (0,0): points inside should
  // be cleared.
  openvdb::Vec3d inside_point(0.0, 0.0, 0.5);
  EXPECT_TRUE(frustums[0]->IsInside(inside_point));

  openvdb::Vec3d outside_point(5.0, 5.0, 0.5);
  EXPECT_FALSE(frustums[0]->IsInside(outside_point));
}

TEST_F(RobotMotionTrackerTest, GenerateInterpolatedCircles) {
  geometry::Circle begin;
  begin.center = Eigen::Vector2d(0.0, 0.0);
  begin.radius = 1.0f;

  geometry::Circle end;
  end.center = Eigen::Vector2d(4.0, 0.0);
  end.radius = 1.0f;

  std::vector<std::unique_ptr<geometry::Circle>> circles;
  tracker_->GenerateInterpolatedCircles(begin, end, 4, circles);

  // num_circles=4 → produces 5 circles (i=0..4)
  ASSERT_EQ(circles.size(), 5u);
  EXPECT_DOUBLE_EQ(circles.front()->center.x(), 0.0);
  EXPECT_DOUBLE_EQ(circles.back()->center.x(), 4.0);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_robot_motion_tracker");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
