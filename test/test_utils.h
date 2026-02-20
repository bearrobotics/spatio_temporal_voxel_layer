#ifndef SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <vector>

#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_frustum.hpp"
#include "spatio_temporal_voxel_layer/robot_motion_tracker.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
namespace spatio_temporal_voxel_layer::test_utils {
using geometry::FootprintFrustum;
using volume_grid::SpatioTemporalVoxelGrid;
// Mock object creators - create disabled instances for testing
std::unique_ptr<FootprintFrustum> MakeMockFootprintFrustum();
std::unique_ptr<DynamicObstacleTracker> MakeMockDynamicObstacleTracker();
std::unique_ptr<RobotMotionTracker> MakeMockRobotMotionTracker();

// Test grid creator - creates a SpatioTemporalVoxelGrid with mock dependencies
std::unique_ptr<SpatioTemporalVoxelGrid> MakeTestGrid();

// Test data helpers
sensor_msgs::PointCloud2 MakePointCloud(
    const std::vector<geometry_msgs::Point>& points);

// Test constants
constexpr float kVoxelSize = 0.05f;
constexpr double kBackgroundValue = 0.0;
constexpr int kDecayModel = 2;  // volume_grid::PERSISTENT
constexpr double kVoxelDecay = 0.0;
constexpr bool kPubVoxels = false;

}  // namespace spatio_temporal_voxel_layer::test_utils

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER_TEST_UTILS_H_
