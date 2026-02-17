#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

observation::MeasurementReading MakeMarkingReading(
    const geometry_msgs::Point& origin,
    const std::vector<geometry_msgs::Point>& obstacle_points) {
  observation::MeasurementReading reading;
  reading._sensor_name = "marker";
  reading._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      spatio_temporal_voxel_layer::test_utils::MakePointCloud(obstacle_points));
  reading._origin = origin;
  reading._marking = true;
  reading._clearing = false;
  reading._obstacle_range_in_m = 100.0;
  return reading;
}

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Test fixture for ClearCircularArea function tests.
// Tests basic functionality of the underlying grid clearing logic.
class ClearRobotFootprintTest : public ::testing::Test {
 protected:
  void SetUp() override {
    grid_ = spatio_temporal_voxel_layer::test_utils::MakeTestGrid();
  }

  void MarkObstacles(const std::vector<geometry_msgs::Point>& obstacles) {
    auto reading = MakeMarkingReading(MakePoint(0.0, 0.0, 0.0), obstacles);
    grid_->Mark({reading});
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
};

// =============================================================================
// Basic Functionality Tests - Verify the function can be called without crashes
// =============================================================================

TEST_F(ClearRobotFootprintTest, ClearEmptyGrid) {
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Should handle clearing empty grid";
}

TEST_F(ClearRobotFootprintTest, ClearWithObstaclesAtOrigin) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
      MakePoint(0.1, 0.1, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Should handle clearing obstacles at origin";
}

TEST_F(ClearRobotFootprintTest, ClearAtNonZeroPosition) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(5.0, 3.0, 1.0),
      MakePoint(5.3, 3.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(5.0, 3.0, 0.5))
      << "Should handle clearing at non-zero position";
}

TEST_F(ClearRobotFootprintTest, ClearAtNegativeCoordinates) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(-2.0, -1.0, 1.0),
      MakePoint(-2.2, -1.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(-2.0, -1.0, 0.5))
      << "Should handle negative coordinates";
}

// =============================================================================
// Radius Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, ZeroRadius) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.0))
      << "Should handle zero radius";
}

TEST_F(ClearRobotFootprintTest, SmallRadius) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.1))
      << "Should handle small radius";
}

TEST_F(ClearRobotFootprintTest, LargeRadius) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
      MakePoint(2.0, 0.0, 1.0),
      MakePoint(0.0, 3.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 5.0))
      << "Should handle large radius";
}

TEST_F(ClearRobotFootprintTest, VeryLargeRadius) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 100.0))
      << "Should handle very large radius";
}

// =============================================================================
// Multiple Obstacle Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, ClearDenseObstacleField) {
  std::vector<geometry_msgs::Point> obstacles;
  for (double x = -2.0; x <= 2.0; x += 0.2) {
    for (double y = -2.0; y <= 2.0; y += 0.2) {
      obstacles.push_back(MakePoint(x, y, 1.0));
    }
  }
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 1.0))
      << "Should handle dense obstacle field";
}

TEST_F(ClearRobotFootprintTest, ClearSparseObstacles) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),   MakePoint(10.0, 0.0, 1.0),
      MakePoint(0.0, 10.0, 1.0),  MakePoint(10.0, 10.0, 1.0),
      MakePoint(-10.0, 0.0, 1.0), MakePoint(0.0, -10.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Should handle sparse obstacles";
}

// =============================================================================
// Height Variation Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, ObstaclesAtDifferentHeights) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 0.1),   // Ground level
      MakePoint(0.0, 0.0, 1.0),   // Mid height
      MakePoint(0.0, 0.0, 2.0),   // High
      MakePoint(0.0, 0.0, -0.5),  // Below ground
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Should handle obstacles at different heights";
}

// =============================================================================
// Geometry Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, ObstaclesInAllQuadrants) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.3, 0.3, 1.0),    // Quadrant 1
      MakePoint(-0.3, 0.3, 1.0),   // Quadrant 2
      MakePoint(-0.3, -0.3, 1.0),  // Quadrant 3
      MakePoint(0.3, -0.3, 1.0),   // Quadrant 4
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Should handle obstacles in all quadrants";
}

TEST_F(ClearRobotFootprintTest, ObstaclesAlongAxes) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.5, 0.0, 1.0),   // +X axis
      MakePoint(-0.5, 0.0, 1.0),  // -X axis
      MakePoint(0.0, 0.5, 1.0),   // +Y axis
      MakePoint(0.0, -0.5, 1.0),  // -Y axis
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.6))
      << "Should handle obstacles along axes";
}

// =============================================================================
// Repeated Operation Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, MultipleClearsAtSameLocation) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5));
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5));
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Multiple clears at same location should not crash";
}

TEST_F(ClearRobotFootprintTest, SequentialClearsAtDifferentLocations) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
      MakePoint(5.0, 0.0, 1.0),
      MakePoint(0.0, 5.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5));
  EXPECT_NO_THROW(grid_->ClearCircularArea(5.0, 0.0, 0.5));
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 5.0, 0.5))
      << "Sequential clears at different locations should work";
}

TEST_F(ClearRobotFootprintTest, AlternatingMarkAndClear) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };

  MarkObstacles(obstacles);
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5));

  MarkObstacles(obstacles);
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5));

  MarkObstacles(obstacles);
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Alternating mark and clear should work";
}

// =============================================================================
// Edge Case Tests
// =============================================================================

TEST_F(ClearRobotFootprintTest, ClearWithNoObstacles) {
  EXPECT_NO_THROW(grid_->ClearCircularArea(0.0, 0.0, 0.5))
      << "Clearing with no obstacles should not crash";
}

TEST_F(ClearRobotFootprintTest, ClearFarFromAnyObstacle) {
  std::vector<geometry_msgs::Point> obstacles = {
      MakePoint(0.0, 0.0, 1.0),
  };
  MarkObstacles(obstacles);

  EXPECT_NO_THROW(grid_->ClearCircularArea(100.0, 100.0, 0.5))
      << "Clearing far from obstacles should work";
}

TEST_F(ClearRobotFootprintTest, ExtremeCoordinates) {
  EXPECT_NO_THROW(grid_->ClearCircularArea(1000.0, 1000.0, 0.5))
      << "Should handle very large coordinates";
  EXPECT_NO_THROW(grid_->ClearCircularArea(-1000.0, -1000.0, 0.5))
      << "Should handle very large negative coordinates";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_clear_robot_footprint_service");
  return RUN_ALL_TESTS();
}
