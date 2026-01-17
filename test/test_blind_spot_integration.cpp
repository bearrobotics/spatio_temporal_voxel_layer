#include <openvdb/openvdb.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace {

constexpr float kVoxelSize = 0.05f;
constexpr double kBackgroundValue = 0.0;
constexpr int kDecayModel = volume_grid::PERSISTENT;
constexpr double kVoxelDecay = 0.0;
constexpr bool kPubVoxels = false;

std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> MakeTestGrid() {
  openvdb::initialize();
  return std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(
      kVoxelSize, kBackgroundValue, kDecayModel, kVoxelDecay, kPubVoxels);
}

sensor_msgs::PointCloud2 MakePointCloud(
    const std::vector<geometry_msgs::Point>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "map";

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& pt : points) {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

observation::MeasurementReading MakeMarkingReading(
    const geometry_msgs::Point& origin,
    const std::vector<geometry_msgs::Point>& obstacle_points) {
  observation::MeasurementReading reading;
  reading._sensor_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      MakePointCloud(obstacle_points));
  reading._origin = origin;
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  return reading;
}

observation::MeasurementReading MakeSensorReading(
    ros::NodeHandle& frustum_nh, const std::string& sensor_name,
    const ros::Time& time, const geometry_msgs::Point& origin,
    const geometry_msgs::Quaternion& orientation) {
  observation::MeasurementReading reading;
  reading._sensor_name = sensor_name;
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>();
  reading._cloud->header.stamp = time;
  reading._origin = origin;
  reading._orientation = orientation;
  reading._frustrum_factory =
      FrustumFactoryFactory::CreateFrustumFactory(frustum_nh);
  reading._marking = false;
  reading._clearing = false;
  return reading;
}

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion MakeIdentityQuaternion() {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

class BlindSpotIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    grid_ = MakeTestGrid();
    frustum_nh_ = ros::NodeHandle("/blind_spot_integration_test/frustum");
    time_ = ros::Time(1.0);
  }

  void MarkObstacle(const geometry_msgs::Point& origin,
                    const std::vector<geometry_msgs::Point>& points) {
    auto reading = MakeMarkingReading(origin, points);
    grid_->Mark({reading});
  }

  void AddSensor(const std::string& name, const geometry_msgs::Point& origin,
                 const geometry_msgs::Quaternion& orientation) {
    auto reading =
        MakeSensorReading(frustum_nh_, name, time_, origin, orientation);
    grid_->UpdateLastReadings(reading);
  }

  void UpdateSensor(const std::string& name, const geometry_msgs::Point& origin,
                    const geometry_msgs::Quaternion& orientation) {
    time_ = time_ + ros::Duration(1.0);
    auto reading =
        MakeSensorReading(frustum_nh_, name, time_, origin, orientation);
    grid_->UpdateLastReadings(reading);
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
  ros::NodeHandle frustum_nh_;
  ros::Time time_;
};

TEST_F(BlindSpotIntegrationTest, SensorCoversObstacle_NoBlindSpot) {
  // Scenario: Robot with forward-facing sensor detects obstacle in front

  // Place obstacle in front of sensor
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Sensor at origin, pointing along +Z
  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check for blind spots in area containing obstacle
  openvdb::Vec3d min_corner(-1.0, -1.0, 1.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 3.0);

  auto blind_spot = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(blind_spot.has_value())
      << "Obstacle should be visible to sensor";
}

TEST_F(BlindSpotIntegrationTest, ObstacleBehindSensor_BlindSpot) {
  // Scenario: Obstacle behind the robot is not visible to forward sensor

  // Place obstacle behind sensor
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, -2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Sensor at origin, pointing along +Z (away from obstacle)
  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check for blind spots in area behind sensor
  openvdb::Vec3d min_corner(-1.0, -1.0, -3.0);
  openvdb::Vec3d max_corner(1.0, 1.0, -1.0);

  auto blind_spot = grid_->CheckBox(min_corner, max_corner);
  ASSERT_TRUE(blind_spot.has_value())
      << "Obstacle behind sensor should be a blind spot";
}

TEST_F(BlindSpotIntegrationTest, MultiSensorCoverage_NoBlindSpot) {
  // Scenario: Multiple sensors at different positions cover different areas

  // Place obstacles in front of each sensor
  geometry_msgs::Point left_obstacle = MakePoint(-5.0, 0.0, 2.0);
  geometry_msgs::Point right_obstacle = MakePoint(5.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {left_obstacle, right_obstacle});

  // Left sensor sees left obstacle
  AddSensor("left_camera", MakePoint(-5.0, 0.0, 0.0), MakeIdentityQuaternion());
  // Right sensor sees right obstacle
  AddSensor("right_camera", MakePoint(5.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check left area - should be covered by left sensor
  openvdb::Vec3d left_min(-6.0, -1.0, 1.0);
  openvdb::Vec3d left_max(-4.0, 1.0, 3.0);
  auto left_blind = grid_->CheckBox(left_min, left_max);
  EXPECT_FALSE(left_blind.has_value())
      << "Left obstacle should be visible to left sensor";

  // Check right area - should be covered by right sensor
  openvdb::Vec3d right_min(4.0, -1.0, 1.0);
  openvdb::Vec3d right_max(6.0, 1.0, 3.0);
  auto right_blind = grid_->CheckBox(right_min, right_max);
  EXPECT_FALSE(right_blind.has_value())
      << "Right obstacle should be visible to right sensor";
}

TEST_F(BlindSpotIntegrationTest, SensorMovement_UpdatesBlindSpots) {
  // Scenario: Sensor moves position and previously blind obstacle becomes
  // visible

  // Place obstacle in front at position (0, 0, 2)
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Initial sensor at a different position - can't see obstacle
  AddSensor("moving_camera", MakePoint(10.0, 0.0, 0.0),
            MakeIdentityQuaternion());

  openvdb::Vec3d obstacle_area_min(-1.0, -1.0, 1.0);
  openvdb::Vec3d obstacle_area_max(1.0, 1.0, 3.0);

  // Before move: obstacle is a blind spot (sensor is far away)
  auto blind_before = grid_->CheckBox(obstacle_area_min, obstacle_area_max);
  ASSERT_TRUE(blind_before.has_value())
      << "Obstacle should be blind spot when sensor is far away";

  // Move sensor to origin where it can see the obstacle
  UpdateSensor("moving_camera", MakePoint(0.0, 0.0, 0.0),
               MakeIdentityQuaternion());

  // After move: obstacle should be visible
  auto blind_after = grid_->CheckBox(obstacle_area_min, obstacle_area_max);
  EXPECT_FALSE(blind_after.has_value())
      << "Obstacle should be visible after sensor moves";
}

TEST_F(BlindSpotIntegrationTest, ObstacleAtFrustumEdge) {
  // Scenario: Test obstacle at the edge of sensor FOV

  // Sensor has horizontal_fov_angle: 1.2 rad (~69 degrees total)
  // At distance 2.0, edge is roughly at x = 2.0 * tan(0.6) = 1.37

  // Place obstacle just inside FOV edge
  geometry_msgs::Point edge_inside = MakePoint(1.2, 0.0, 2.0);
  // Place obstacle just outside FOV edge
  geometry_msgs::Point edge_outside = MakePoint(2.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {edge_inside, edge_outside});

  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check the area containing inside-edge obstacle
  openvdb::Vec3d inside_min(1.0, -0.5, 1.5);
  openvdb::Vec3d inside_max(1.5, 0.5, 2.5);
  auto inside_blind = grid_->CheckBox(inside_min, inside_max);
  EXPECT_FALSE(inside_blind.has_value())
      << "Obstacle inside FOV edge should be visible";

  // Check the area containing outside-edge obstacle
  openvdb::Vec3d outside_min(1.8, -0.5, 1.5);
  openvdb::Vec3d outside_max(2.5, 0.5, 2.5);
  auto outside_blind = grid_->CheckBox(outside_min, outside_max);
  EXPECT_TRUE(outside_blind.has_value())
      << "Obstacle outside FOV edge should be blind spot";
}

TEST_F(BlindSpotIntegrationTest, ObstacleAtRangeLimit) {
  // Scenario: Test obstacles at sensor's min and max range

  // min_z: 0.1, max_z: 5.0 from launch file

  // Place obstacle at min range
  geometry_msgs::Point min_range_obstacle = MakePoint(0.0, 0.0, 0.1);
  // Place obstacle beyond max range
  geometry_msgs::Point beyond_max_obstacle = MakePoint(0.0, 0.0, 6.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0),
               {min_range_obstacle, beyond_max_obstacle});

  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check area at min range
  openvdb::Vec3d min_min(-0.5, -0.5, 0.0);
  openvdb::Vec3d min_max(0.5, 0.5, 0.5);
  auto min_blind = grid_->CheckBox(min_min, min_max);
  EXPECT_FALSE(min_blind.has_value())
      << "Obstacle at min range should be visible";

  // Check area beyond max range
  openvdb::Vec3d beyond_min(-0.5, -0.5, 5.5);
  openvdb::Vec3d beyond_max(0.5, 0.5, 7.0);
  auto beyond_blind = grid_->CheckBox(beyond_min, beyond_max);
  EXPECT_TRUE(beyond_blind.has_value())
      << "Obstacle beyond max range should be blind spot";
}

TEST_F(BlindSpotIntegrationTest, MultipleSensorsPartialCoverage) {
  // Scenario: Multiple sensors but blind spot exists between them

  // Place obstacles in a row
  MarkObstacle(MakePoint(0.0, 0.0, 0.0),
               {MakePoint(-3.0, 0.0, 2.0), MakePoint(0.0, 0.0, 2.0),
                MakePoint(3.0, 0.0, 2.0)});

  // Left sensor can see left obstacle
  AddSensor("left_camera", MakePoint(-3.0, 0.0, 0.0), MakeIdentityQuaternion());
  // Right sensor can see right obstacle
  AddSensor("right_camera", MakePoint(3.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Middle obstacle might be in a blind spot between the two sensors
  openvdb::Vec3d mid_min(-1.0, -0.5, 1.5);
  openvdb::Vec3d mid_max(1.0, 0.5, 2.5);
  auto mid_blind = grid_->CheckBox(mid_min, mid_max);

  // Center obstacle should be visible to at least one sensor if within their
  // FOV But since sensors are at -3 and +3, and FOV is ~1.2 rad, center might
  // be visible This test documents the actual behavior
  // The center at distance 2.0 from z=0 line, with sensors at x=+-3
  // Distance from left sensor to center obstacle: sqrt(9+4) = 3.6m
  // Angle from left sensor to center: atan(3/2) = 0.98 rad
  // With hFOV of 1.2 rad (0.6 on each side), center is outside FOV
  EXPECT_TRUE(mid_blind.has_value())
      << "Middle obstacle should be blind spot between two sensors";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_blind_spot_integration");
  return RUN_ALL_TESTS();
}
