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

class CheckBoxTest : public ::testing::Test {
 protected:
  void SetUp() override {
    grid_ = MakeTestGrid();
    frustum_nh_ = ros::NodeHandle("/check_box_test/frustum");
  }

  void MarkObstacle(const geometry_msgs::Point& origin,
                    const std::vector<geometry_msgs::Point>& points) {
    auto reading = MakeMarkingReading(origin, points);
    grid_->Mark({reading});
  }

  void AddSensor(const std::string& name, const geometry_msgs::Point& origin,
                 const geometry_msgs::Quaternion& orientation) {
    auto reading = MakeSensorReading(frustum_nh_, name, ros::Time::now(),
                                     origin, orientation);
    grid_->UpdateLastReadings(reading);
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
  ros::NodeHandle frustum_nh_;
};

TEST_F(CheckBoxTest, EmptyGrid_ReturnsNullopt) {
  // No obstacles marked in the grid
  openvdb::Vec3d min_corner(0.0, 0.0, 0.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 1.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CheckBoxTest, NoVoxelsInBox_ReturnsNullopt) {
  // Mark obstacle outside the query box
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {MakePoint(10.0, 10.0, 1.0)});

  // Query box doesn't contain the obstacle
  openvdb::Vec3d min_corner(0.0, 0.0, 0.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 1.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CheckBoxTest, VoxelInBox_InFrustum_ReturnsNullopt) {
  // Mark obstacle in front of sensor (within FOV)
  // Sensor at origin, looking along +Z
  geometry_msgs::Point obstacle_pos = MakePoint(0.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle_pos});

  // Add sensor that can see the obstacle
  AddSensor("sensor_a", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Query box contains the obstacle
  openvdb::Vec3d min_corner(-1.0, -1.0, 1.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 3.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  // Obstacle is visible to sensor, so no blind spot
  EXPECT_FALSE(result.has_value());
}

TEST_F(CheckBoxTest, VoxelInBox_NotInFrustum_ReturnsPoint) {
  // Mark obstacle behind sensor (not in FOV)
  // Sensor at origin, looking along +Z, obstacle is at negative Z
  geometry_msgs::Point obstacle_pos = MakePoint(0.0, 0.0, -2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle_pos});

  // Add sensor that CANNOT see the obstacle (looking the wrong way)
  AddSensor("sensor_a", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Query box contains the obstacle
  openvdb::Vec3d min_corner(-1.0, -1.0, -3.0);
  openvdb::Vec3d max_corner(1.0, 1.0, -1.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  // Obstacle is NOT visible to sensor, so it's a blind spot
  ASSERT_TRUE(result.has_value());

  // Verify returned point is near the obstacle
  EXPECT_NEAR(result->x(), obstacle_pos.x, kVoxelSize * 2);
  EXPECT_NEAR(result->y(), obstacle_pos.y, kVoxelSize * 2);
  EXPECT_NEAR(result->z(), obstacle_pos.z, kVoxelSize * 2);
}

TEST_F(CheckBoxTest, VoxelInBox_NoSensors_ReturnsPoint) {
  // Mark obstacle with no sensors registered
  geometry_msgs::Point obstacle_pos = MakePoint(1.0, 1.0, 1.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle_pos});

  // Query box contains the obstacle
  openvdb::Vec3d min_corner(0.0, 0.0, 0.0);
  openvdb::Vec3d max_corner(2.0, 2.0, 2.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  // No sensors, so all obstacles are blind spots
  ASSERT_TRUE(result.has_value());
}

TEST_F(CheckBoxTest, MultipleBlindSpots_ReturnsOne) {
  // Mark multiple obstacles not in FOV
  MarkObstacle(MakePoint(0.0, 0.0, 0.0),
               {MakePoint(0.0, 0.0, -2.0), MakePoint(0.0, 0.0, -3.0),
                MakePoint(0.0, 0.0, -4.0)});

  // Add sensor looking the wrong way
  AddSensor("sensor_a", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  openvdb::Vec3d min_corner(-1.0, -1.0, -5.0);
  openvdb::Vec3d max_corner(1.0, 1.0, -1.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  // Should return one of the blind spots
  ASSERT_TRUE(result.has_value());

  // The returned point should be one of the obstacles
  EXPECT_NEAR(result->x(), 0.0, kVoxelSize * 2);
  EXPECT_NEAR(result->y(), 0.0, kVoxelSize * 2);
  // Z should be one of -2, -3, or -4
  EXPECT_LT(result->z(), -1.0);
  EXPECT_GT(result->z(), -5.0);
}

TEST_F(CheckBoxTest, MixedVisibility_ReturnsBlindSpotOnly) {
  // Mark one obstacle in FOV and one outside FOV
  geometry_msgs::Point visible_obstacle = MakePoint(0.0, 0.0, 2.0);
  geometry_msgs::Point blind_obstacle = MakePoint(0.0, 0.0, -2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {visible_obstacle, blind_obstacle});

  // Add sensor
  AddSensor("sensor_a", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Query box contains only the blind spot obstacle
  openvdb::Vec3d min_corner(-1.0, -1.0, -3.0);
  openvdb::Vec3d max_corner(1.0, 1.0, -1.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->z(), blind_obstacle.z, kVoxelSize * 2);
}

TEST_F(CheckBoxTest, ObstacleOutsideSensorRange_IsBlindSpot) {
  // Mark obstacle beyond sensor's max range (max_z = 5.0)
  geometry_msgs::Point far_obstacle = MakePoint(0.0, 0.0, 10.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {far_obstacle});

  // Add sensor (max_z = 5.0 in launch file)
  AddSensor("sensor_a", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  openvdb::Vec3d min_corner(-1.0, -1.0, 8.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 12.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  // Obstacle is too far for sensor, so it's a blind spot
  ASSERT_TRUE(result.has_value());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_check_box");
  return RUN_ALL_TESTS();
}
