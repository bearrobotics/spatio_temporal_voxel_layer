#include <openvdb/openvdb.h>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_reading.hpp"
#include "spatio_temporal_voxel_layer/dynamic_obstacle_tracker.hpp"

namespace {

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

TEST(PolygonIsInsideTest, PointInside) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point(0.0, 0.0, 0.0);
  EXPECT_TRUE(polygon.IsInside(point));
}

TEST(PolygonIsInsideTest, PointOutside) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point(5.0, 5.0, 0.0);
  EXPECT_FALSE(polygon.IsInside(point));
}

TEST(PolygonIsInsideTest, ZIgnored) {
  DynamicObstacleTracker::Polygon polygon;
  polygon.polygon_ = MakeSquarePolygon(0.0f, 0.0f, 2.0f);

  openvdb::Vec3d point_high_z(0.0, 0.0, 1000.0);
  EXPECT_TRUE(polygon.IsInside(point_high_z));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
