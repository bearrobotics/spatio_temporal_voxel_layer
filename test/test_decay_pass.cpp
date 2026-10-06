#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <openvdb/openvdb.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using volume_grid::occupany_cell;
using volume_grid::SpatioTemporalVoxelGrid;

// LINEAR decay expires a voxel once voxel_decay minus its age drops below zero.
constexpr double kDecayExpired = -1.0;
constexpr double kDecayNever = 1000.0;
constexpr double kFrustumAcceleration = 1.0;
constexpr double kPrismAcceleration = 2.0;
constexpr double kQueryTolerance = 0.07;

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

// The camera at the origin looks along +z, so negative z lies outside it.
const geometry_msgs::Point kInFrontOfCamera = MakePoint(0.3, 0.0, 1.0);
const geometry_msgs::Point kBehindCamera = MakePoint(0.3, 0.6, -1.0);
const geometry_msgs::Point kFarFromCamera = MakePoint(1.3, 0.0, 1.0);
// Inside the polygon that MakeInterSensorDecayPrism defines.
const geometry_msgs::Point kInsidePrism = MakePoint(0.3, 0.0, 0.5);

std::unique_ptr<geometry::InterSensorDecayPrism> MakeInterSensorDecayPrism() {
  ros::NodeHandle nh;
  geometry::InterSensorDecayPrism::Config config;
  config.prism_config.enable = true;
  config.prism_config.publish_visualization = false;
  config.prism_config.min_z = 0.1;
  config.prism_config.max_z = 1.0;
  config.prism_config.footprint_points = {
      geometry::FootprintClearingPrism::Point(0.0, 0.2),
      geometry::FootprintClearingPrism::Point(0.4, 0.2),
      geometry::FootprintClearingPrism::Point(0.4, -0.2),
      geometry::FootprintClearingPrism::Point(0.0, -0.2),
  };
  config.decay_acceleration_factor = kPrismAcceleration;
  return geometry::InterSensorDecayPrism::Create(config, nh);
}

std::unique_ptr<SpatioTemporalVoxelGrid> MakeLinearDecayGrid(
    double voxel_decay,
    std::unique_ptr<geometry::InterSensorDecayPrism> inter_sensor_decay_prism =
        nullptr) {
  openvdb::initialize();
  auto grid = std::make_unique<SpatioTemporalVoxelGrid>(
      test_utils::kVoxelSize, test_utils::kBackgroundValue, volume_grid::LINEAR,
      voxel_decay, test_utils::kPubVoxels,
      test_utils::MakeMockFootprintFrustum(),
      std::move(inter_sensor_decay_prism),
      /*front_blind_spot_clearing_prism=*/nullptr,
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker());
  grid->SetRobotPose(0.0, 0.0, 0.0);
  return grid;
}

observation::MeasurementReading MakeMarkingReading(
    const std::vector<geometry_msgs::Point>& points) {
  observation::MeasurementReading reading;
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(points));
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  reading._decay_acceleration = 0.0;
  return reading;
}

observation::MeasurementReading MakeClearingReading(
    ros::NodeHandle& frustum_nh) {
  observation::MeasurementReading reading;
  reading._sensor_name = "camera";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>();
  reading._cloud->header.stamp = ros::Time(1.0);
  reading._origin = MakePoint(0.0, 0.0, 0.0);
  reading._orientation = MakeIdentityQuaternion();
  reading._frustrum_factory =
      FrustumFactoryFactory::CreateFrustumFactory(frustum_nh);
  reading._marking = false;
  reading._clearing = true;
  reading._obstacle_range_in_m = 100.0;
  reading._decay_acceleration = kFrustumAcceleration;
  return reading;
}

std::unordered_set<occupany_cell> RunDecayPass(
    SpatioTemporalVoxelGrid& grid,
    const std::vector<observation::MeasurementReading>& clearing_readings) {
  std::unordered_set<occupany_cell> cleared_cells;
  std::vector<DynamicObstacleReading> dynamic_obstacle_readings;
  std::vector<RobotMotionReading> robot_motion_readings;
  grid.ClearFrustums(clearing_readings, cleared_cells,
                     dynamic_obstacle_readings, robot_motion_readings);
  return cleared_cells;
}

bool HasVoxelNear(const SpatioTemporalVoxelGrid& grid,
                  const geometry_msgs::Point& point) {
  return !grid.GetVoxelsAtXY(point.x, point.y, kQueryTolerance).empty();
}

class DecayPassTest : public ::testing::Test {
 protected:
  void SetUp() override {
    frustum_nh_ = ros::NodeHandle("/decay_pass_test/frustum");
  }

  ros::NodeHandle frustum_nh_;
};

TEST_F(DecayPassTest, SurvivingVoxelIsCountedInCostmap) {
  auto grid = MakeLinearDecayGrid(kDecayNever);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera})});

  const std::unordered_set<occupany_cell> cleared = RunDecayPass(*grid, {});

  EXPECT_TRUE(cleared.empty());
  EXPECT_TRUE(HasVoxelNear(*grid, kInFrontOfCamera));
  ASSERT_EQ(grid->GetFlattenedCostmap()->size(), 1u);
  EXPECT_EQ(grid->GetFlattenedCostmap()->begin()->second, 1u);
}

TEST_F(DecayPassTest, ExpiredVoxelIsClearedByTemporalDecay) {
  auto grid = MakeLinearDecayGrid(kDecayExpired);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera})});

  const std::unordered_set<occupany_cell> cleared = RunDecayPass(*grid, {});

  EXPECT_EQ(cleared.size(), 1u);
  EXPECT_FALSE(HasVoxelNear(*grid, kInFrontOfCamera));
  EXPECT_TRUE(grid->GetFlattenedCostmap()->empty());
}

TEST_F(DecayPassTest, FrustumLeavesUnexpiredVoxelInPlace) {
  auto grid = MakeLinearDecayGrid(kDecayNever);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera, kBehindCamera})});

  const std::unordered_set<occupany_cell> cleared =
      RunDecayPass(*grid, {MakeClearingReading(frustum_nh_)});

  EXPECT_TRUE(cleared.empty());
  EXPECT_TRUE(HasVoxelNear(*grid, kInFrontOfCamera));
  EXPECT_TRUE(HasVoxelNear(*grid, kBehindCamera));
  EXPECT_EQ(grid->GetFlattenedCostmap()->size(), 2u);
}

TEST_F(DecayPassTest, ExpiredVoxelsAreClearedInsideAndOutsideFrustum) {
  auto grid = MakeLinearDecayGrid(kDecayExpired);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera, kBehindCamera})});

  const std::unordered_set<occupany_cell> cleared =
      RunDecayPass(*grid, {MakeClearingReading(frustum_nh_)});

  EXPECT_EQ(cleared.size(), 2u);
  EXPECT_FALSE(HasVoxelNear(*grid, kInFrontOfCamera));
  EXPECT_FALSE(HasVoxelNear(*grid, kBehindCamera));
  EXPECT_TRUE(grid->GetFlattenedCostmap()->empty());
}

TEST_F(DecayPassTest, InterSensorPrismLeavesUnexpiredVoxelInPlace) {
  auto grid = MakeLinearDecayGrid(kDecayNever, MakeInterSensorDecayPrism());
  grid->Mark({MakeMarkingReading({kInsidePrism})});

  const std::unordered_set<occupany_cell> cleared = RunDecayPass(*grid, {});

  EXPECT_TRUE(cleared.empty());
  EXPECT_TRUE(HasVoxelNear(*grid, kInsidePrism));
}

TEST_F(DecayPassTest, InterSensorPrismClearsExpiredVoxel) {
  auto grid = MakeLinearDecayGrid(kDecayExpired, MakeInterSensorDecayPrism());
  grid->Mark({MakeMarkingReading({kInsidePrism})});

  const std::unordered_set<occupany_cell> cleared = RunDecayPass(*grid, {});

  EXPECT_EQ(cleared.size(), 1u);
  EXPECT_FALSE(HasVoxelNear(*grid, kInsidePrism));
}

TEST_F(DecayPassTest, ResetGridAreaKeepsTheBoxAndClearsOutside) {
  auto grid = MakeLinearDecayGrid(kDecayNever);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera, kFarFromCamera})});

  grid->ResetGridArea(occupany_cell(0.0, -0.5), occupany_cell(0.6, 0.5));

  EXPECT_TRUE(HasVoxelNear(*grid, kInFrontOfCamera));
  EXPECT_FALSE(HasVoxelNear(*grid, kFarFromCamera));
}

TEST_F(DecayPassTest, ResetGridAreaInvertedClearsInsideTheBox) {
  auto grid = MakeLinearDecayGrid(kDecayNever);
  grid->Mark({MakeMarkingReading({kInFrontOfCamera, kFarFromCamera})});

  grid->ResetGridArea(occupany_cell(0.0, -0.5), occupany_cell(0.6, 0.5),
                      /*invert_area=*/true);

  EXPECT_FALSE(HasVoxelNear(*grid, kInFrontOfCamera));
  EXPECT_TRUE(HasVoxelNear(*grid, kFarFromCamera));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_decay_pass");
  return RUN_ALL_TESTS();
}
