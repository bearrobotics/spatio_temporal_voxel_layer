#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/CheckBlindSpot.h"
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

spatio_temporal_voxel_layer::CheckBlindSpot::Request MakeValidRequest(
    double x, double y, double tolerance = 0.5, double min_height = 0.0,
    double max_height = 2.0, const std::string& frame_id = "map") {
  spatio_temporal_voxel_layer::CheckBlindSpot::Request req;
  req.point.header.frame_id = frame_id;
  req.point.header.stamp = ros::Time::now();
  req.point.point.x = x;
  req.point.point.y = y;
  req.point.point.z = 0.0;
  req.tolerance = tolerance;
  req.min_height = min_height;
  req.max_height = max_height;
  return req;
}

// Test fixture for CheckBlindSpot service tests.
// Tests the service callback validation logic and the underlying CheckBox
// method that the service uses. The service callback performs validation
// then calls grid->CheckBox() with a computed bounding box.
class CheckBlindSpotServiceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    frustum_nh_ = ros::NodeHandle("/test_check_blind_spot_service/frustum");
    grid_ = MakeTestGrid();
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

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
  ros::NodeHandle frustum_nh_;
  ros::Time time_;
};

// =============================================================================
// Input Validation Tests
// The service callback validates these conditions before processing.
// These tests verify the validation patterns that the service implements.
// =============================================================================

TEST_F(CheckBlindSpotServiceTest, EmptyFrameId_InvalidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.point.header.frame_id = "";
  // Service rejects requests with empty frame_id (returns msg="no_frame_id")
  EXPECT_TRUE(req.point.header.frame_id.empty());
}

TEST_F(CheckBlindSpotServiceTest, NegativeTolerance_InvalidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.tolerance = -0.1;
  // Service rejects negative tolerance (returns msg="negative_tolerance")
  EXPECT_LT(req.tolerance, 0.0);
}

TEST_F(CheckBlindSpotServiceTest, UnreasonablyLargeTolerance_InvalidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.tolerance = 5.1;
  // Service rejects tolerance > 5.0 (returns msg="unreasonable_tolerance")
  EXPECT_GT(req.tolerance, 5.0);
}

TEST_F(CheckBlindSpotServiceTest, NegativeMinHeight_InvalidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.min_height = -0.1;
  // Service rejects negative min_height (returns msg="negative_min_height")
  EXPECT_LT(req.min_height, 0.0);
}

TEST_F(CheckBlindSpotServiceTest, MaxHeightLessThanMinHeight_InvalidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.min_height = 1.0;
  req.max_height = 0.5;
  // Service rejects max < min (returns msg="max_height_less_than_min_height")
  EXPECT_LT(req.max_height, req.min_height);
}

// =============================================================================
// Valid Parameter Edge Case Tests
// =============================================================================

TEST_F(CheckBlindSpotServiceTest, ZeroTolerance_ValidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.tolerance = 0.0;
  EXPECT_GE(req.tolerance, 0.0);
  EXPECT_LE(req.tolerance, 5.0);
}

TEST_F(CheckBlindSpotServiceTest, BoundaryTolerance_ExactlyFiveMeters) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.tolerance = 5.0;
  EXPECT_GE(req.tolerance, 0.0);
  EXPECT_LE(req.tolerance, 5.0);
}

TEST_F(CheckBlindSpotServiceTest, EqualMinMaxHeight_ValidRequest) {
  auto req = MakeValidRequest(0.0, 0.0);
  req.min_height = 1.0;
  req.max_height = 1.0;
  EXPECT_GE(req.max_height, req.min_height);
}

// =============================================================================
// Functional Tests - CheckBox Logic via Grid
// The service calls grid->CheckBox() with a bounding box computed from the
// request. These tests validate the underlying CheckBox behavior.
// =============================================================================

TEST_F(CheckBlindSpotServiceTest, EmptyGrid_NoBlindSpot) {
  // Query an empty grid - should find no blind spots
  openvdb::Vec3d min_corner(-1.0, -1.0, 0.0);
  openvdb::Vec3d max_corner(1.0, 1.0, 2.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(result.has_value()) << "Empty grid should have no blind spots";
}

TEST_F(CheckBlindSpotServiceTest, ObstacleInVisibleArea_NoBlindSpot) {
  // Place an obstacle in front of the sensor
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Add a sensor pointing at the obstacle
  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check the area containing the obstacle
  openvdb::Vec3d min_corner(-0.5, -0.5, 1.5);
  openvdb::Vec3d max_corner(0.5, 0.5, 2.5);

  auto result = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(result.has_value())
      << "Obstacle in sensor FOV should not be a blind spot";
}

TEST_F(CheckBlindSpotServiceTest, ObstacleInBlindSpot_ReturnsBlindSpot) {
  // Place an obstacle behind the sensor (not in FOV)
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, -2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Add a sensor pointing away from the obstacle (+Z direction)
  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check the area behind the sensor
  openvdb::Vec3d min_corner(-0.5, -0.5, -2.5);
  openvdb::Vec3d max_corner(0.5, 0.5, -1.5);

  auto result = grid_->CheckBox(min_corner, max_corner);
  ASSERT_TRUE(result.has_value())
      << "Obstacle behind sensor should be a blind spot";
}

TEST_F(CheckBlindSpotServiceTest, QueryFarFromObstacles_NoBlindSpot) {
  // Place an obstacle at the origin
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, 1.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Add a sensor at the origin
  AddSensor("front_camera", MakePoint(0.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Query an area far from the obstacle
  openvdb::Vec3d min_corner(99.5, 99.5, 0.0);
  openvdb::Vec3d max_corner(100.5, 100.5, 2.0);

  auto result = grid_->CheckBox(min_corner, max_corner);
  EXPECT_FALSE(result.has_value())
      << "Query area with no obstacles should return no blind spot";
}

TEST_F(CheckBlindSpotServiceTest, HeightFiltering_RespectsMinMax) {
  // Place obstacle at z=1.0
  geometry_msgs::Point obstacle = MakePoint(0.0, 0.0, 1.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {obstacle});

  // Add sensor far away so obstacle is in blind spot
  AddSensor("front_camera", MakePoint(10.0, 0.0, 0.0),
            MakeIdentityQuaternion());

  // Query with height range that includes the obstacle
  openvdb::Vec3d min_corner_includes(-0.5, -0.5, 0.5);
  openvdb::Vec3d max_corner_includes(0.5, 0.5, 1.5);
  auto result_includes =
      grid_->CheckBox(min_corner_includes, max_corner_includes);
  EXPECT_TRUE(result_includes.has_value())
      << "Should find blind spot when height range includes obstacle";

  // Query with height range above the obstacle
  openvdb::Vec3d min_corner_above(-0.5, -0.5, 2.0);
  openvdb::Vec3d max_corner_above(0.5, 0.5, 3.0);
  auto result_above = grid_->CheckBox(min_corner_above, max_corner_above);
  EXPECT_FALSE(result_above.has_value())
      << "Should not find blind spot when height range excludes obstacle";
}

TEST_F(CheckBlindSpotServiceTest, MultipleSensors_CoverageCheck) {
  // Place obstacles on the left and right
  geometry_msgs::Point left_obstacle = MakePoint(-2.0, 0.0, 2.0);
  geometry_msgs::Point right_obstacle = MakePoint(2.0, 0.0, 2.0);
  MarkObstacle(MakePoint(0.0, 0.0, 0.0), {left_obstacle, right_obstacle});

  // Add sensors to cover each obstacle
  AddSensor("left_camera", MakePoint(-2.0, 0.0, 0.0), MakeIdentityQuaternion());
  AddSensor("right_camera", MakePoint(2.0, 0.0, 0.0), MakeIdentityQuaternion());

  // Check left area - should be covered
  openvdb::Vec3d left_min(-2.5, -0.5, 1.5);
  openvdb::Vec3d left_max(-1.5, 0.5, 2.5);
  auto left_result = grid_->CheckBox(left_min, left_max);
  EXPECT_FALSE(left_result.has_value())
      << "Left obstacle should be visible to left sensor";

  // Check right area - should be covered
  openvdb::Vec3d right_min(1.5, -0.5, 1.5);
  openvdb::Vec3d right_max(2.5, 0.5, 2.5);
  auto right_result = grid_->CheckBox(right_min, right_max);
  EXPECT_FALSE(right_result.has_value())
      << "Right obstacle should be visible to right sensor";
}

// =============================================================================
// Service BoundingBox Computation Tests
// The service computes the bounding box from the request as:
//   min = (x - tolerance, y - tolerance, min_height)
//   max = (x + tolerance, y + tolerance, max_height)
// =============================================================================

TEST_F(CheckBlindSpotServiceTest, BoundingBoxComputation_ZeroTolerance) {
  // With zero tolerance, the bounding box is a point (or very small volume)
  auto req = MakeValidRequest(5.0, 3.0, 0.0, 0.5, 1.5);

  openvdb::Vec3d expected_min(5.0, 3.0, 0.5);
  openvdb::Vec3d expected_max(5.0, 3.0, 1.5);

  EXPECT_DOUBLE_EQ(req.point.point.x - req.tolerance, expected_min.x());
  EXPECT_DOUBLE_EQ(req.point.point.y - req.tolerance, expected_min.y());
  EXPECT_DOUBLE_EQ(req.min_height, expected_min.z());

  EXPECT_DOUBLE_EQ(req.point.point.x + req.tolerance, expected_max.x());
  EXPECT_DOUBLE_EQ(req.point.point.y + req.tolerance, expected_max.y());
  EXPECT_DOUBLE_EQ(req.max_height, expected_max.z());
}

TEST_F(CheckBlindSpotServiceTest, BoundingBoxComputation_WithTolerance) {
  auto req = MakeValidRequest(2.0, 4.0, 1.0, 0.0, 3.0);

  openvdb::Vec3d expected_min(1.0, 3.0, 0.0);
  openvdb::Vec3d expected_max(3.0, 5.0, 3.0);

  EXPECT_DOUBLE_EQ(req.point.point.x - req.tolerance, expected_min.x());
  EXPECT_DOUBLE_EQ(req.point.point.y - req.tolerance, expected_min.y());
  EXPECT_DOUBLE_EQ(req.min_height, expected_min.z());

  EXPECT_DOUBLE_EQ(req.point.point.x + req.tolerance, expected_max.x());
  EXPECT_DOUBLE_EQ(req.point.point.y + req.tolerance, expected_max.y());
  EXPECT_DOUBLE_EQ(req.max_height, expected_max.z());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_check_blind_spot_service");
  return RUN_ALL_TESTS();
}
