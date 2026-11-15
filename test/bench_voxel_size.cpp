/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2026, Bear Robotics, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 2.1 of the
 *  License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301 USA
 *********************************************************************/

// Benchmark: per-update-cycle CPU cost of the STVL grid vs voxel size and
// input point spacing, on a synthetic restaurant scene shaped like Servi-Q's
// (pennybot_4) configuration: 3 depth-camera clearing frustums, the
// inter-sensor decay prism, the front blind-spot prism, voxel cloud publish.
//
// Not a correctness test; run with:
//   bazel test -c opt
//   //ROS/external/navigation/spatio_temporal_voxel_layer:bench_voxel_size
//     --test_output=streamed

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <openvdb/openvdb.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/frustum_factory.h"
#include "spatio_temporal_voxel_layer/frustum_models/footprint_clearing_prism.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/inter_sensor_decay_prism.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

namespace test_utils = spatio_temporal_voxel_layer::test_utils;
using geometry::FootprintClearingPrism;
using geometry::InterSensorDecayPrism;
using geometry_msgs::Point;
using volume_grid::occupany_cell;
using volume_grid::SpatioTemporalVoxelGrid;

constexpr double kMinZ = 0.03;
constexpr double kMaxZ = 1.24;  // obstacle_detect_max_height-ish
constexpr double kVoxelDecay = 15.0;
constexpr double kCameraRange = 4.0;
constexpr int kTimedCycles = 50;
constexpr int kWarmupCycles = 10;

Point P(double x, double y, double z) {
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// A vertical rectangle standing on the segment (x0, y0)-(x1, y1): a wall or a
// chair back.
struct VerticalPlane {
  double x0, y0, x1, y1;
  double z0, z1;
};

// A horizontal rectangle centred on (cx, cy): a table or seat top.
struct HorizontalPlate {
  double cx, cy;
  double half_x, half_y;
  double z;
};

// A vertical cylinder: a table or chair leg, or a person.
struct Cylinder {
  double cx, cy;
  double radius;
  double z0, z1;
};

std::vector<Point> Sample(const VerticalPlane& plane, double spacing) {
  std::vector<Point> out;
  const double len = std::hypot(plane.x1 - plane.x0, plane.y1 - plane.y0);
  const int n = std::max(1, static_cast<int>(len / spacing));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    for (double z = std::max(plane.z0, kMinZ); z <= std::min(plane.z1, kMaxZ);
         z += spacing) {
      out.push_back(P(plane.x0 + t * (plane.x1 - plane.x0),
                      plane.y0 + t * (plane.y1 - plane.y0), z));
    }
  }
  return out;
}

std::vector<Point> Sample(const HorizontalPlate& plate, double spacing) {
  std::vector<Point> out;
  if (plate.z < kMinZ || plate.z > kMaxZ) return out;
  for (double x = plate.cx - plate.half_x; x <= plate.cx + plate.half_x;
       x += spacing) {
    for (double y = plate.cy - plate.half_y; y <= plate.cy + plate.half_y;
         y += spacing) {
      out.push_back(P(x, y, plate.z));
    }
  }
  return out;
}

std::vector<Point> Sample(const Cylinder& cylinder, double spacing) {
  std::vector<Point> out;
  const int n =
      std::max(4, static_cast<int>(2 * M_PI * cylinder.radius / spacing));
  for (int i = 0; i < n; ++i) {
    const double a = 2 * M_PI * i / n;
    for (double z = std::max(cylinder.z0, kMinZ);
         z <= std::min(cylinder.z1, kMaxZ); z += spacing) {
      out.push_back(P(cylinder.cx + cylinder.radius * std::cos(a),
                      cylinder.cy + cylinder.radius * std::sin(a), z));
    }
  }
  return out;
}

// 12 m x 8 m dining room around the robot, roughly what accumulates over the
// 15 s decay window while driving.
std::vector<Point> MakeScene(double spacing) {
  std::vector<Point> pts;
  const auto add = [&pts, spacing](const auto& shape) {
    const std::vector<Point> sampled = Sample(shape, spacing);
    pts.insert(pts.end(), sampled.begin(), sampled.end());
  };
  add(VerticalPlane{-6, 4, 6, 4, 0, 2});
  add(VerticalPlane{-6, -4, 6, -4, 0, 2});
  add(VerticalPlane{6, -4, 6, 4, 0, 2});
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> ux(-5.5, 5.5), uy(-3.5, 3.5);
  for (int t = 0; t < 12; ++t) {
    const double cx = ux(rng), cy = uy(rng);
    if (std::hypot(cx, cy) < 1.0) continue;  // keep the robot's spot free
    add(HorizontalPlate{cx, cy, 0.4, 0.4, 0.75});
    for (int lx : {-1, 1})
      for (int ly : {-1, 1})
        add(Cylinder{cx + lx * 0.35, cy + ly * 0.35, 0.025, 0, 0.75});
    for (int c = 0; c < 4; ++c) {
      const double a = c * M_PI / 2;
      const double chx = cx + 0.65 * std::cos(a), chy = cy + 0.65 * std::sin(a);
      add(HorizontalPlate{chx, chy, 0.2, 0.2, 0.45});
      add(VerticalPlane{chx - 0.2, chy + 0.2, chx + 0.2, chy + 0.2, 0.45, 0.9});
      for (int lx : {-1, 1})
        for (int ly : {-1, 1})
          add(Cylinder{chx + lx * 0.18, chy + ly * 0.18, 0.015, 0, 0.45});
    }
  }
  for (int h = 0; h < 8; ++h) {
    const double cx = ux(rng), cy = uy(rng);
    if (std::hypot(cx, cy) < 1.0) continue;
    add(Cylinder{cx, cy, 0.2, 0, 1.8});
  }
  return pts;
}

// A forward wedge in the robot frame: what one camera sees this cycle.
struct ViewWedge {
  double yaw;
  double half_fov;
  double min_range = 0.2;
  double max_range = kCameraRange;
};

std::vector<Point> CameraView(const std::vector<Point>& scene,
                              const ViewWedge& wedge) {
  std::vector<Point> out;
  for (const Point& p : scene) {
    const double r = std::hypot(p.x, p.y);
    if (r > wedge.max_range || r < wedge.min_range) continue;
    double a = std::atan2(p.y, p.x) - wedge.yaw;
    a = std::atan2(std::sin(a), std::cos(a));
    if (std::fabs(a) < wedge.half_fov) out.push_back(p);
  }
  return out;
}

geometry_msgs::Quaternion OpticalLookingForward(double pitch) {
  // optical: z forward, x right, y down
  tf2::Quaternion q_opt;
  q_opt.setValue(-0.5, 0.5, -0.5, 0.5);
  tf2::Quaternion q_tilt;
  q_tilt.setRPY(0, pitch, 0);
  return tf2::toMsg(q_tilt * q_opt);
}

observation::MeasurementReading MakeReading(ros::NodeHandle& frustum_nh,
                                            const std::string& name,
                                            const std::vector<Point>& pts,
                                            double z, double pitch,
                                            uint64_t seq) {
  observation::MeasurementReading r;
  r._source_name = name;
  r._cloud = boost::make_shared<sensor_msgs::PointCloud2>(
      test_utils::MakePointCloud(pts));
  r._cloud->header.stamp = ros::Time::now();
  r._origin = P(0.2, 0.0, z);
  r._orientation = OpticalLookingForward(pitch);
  r._frustrum_factory = FrustumFactoryFactory::CreateFrustumFactory(frustum_nh);
  r._marking = true;
  r._clearing = true;
  r._obstacle_range_in_m = 6.0;
  r._decay_acceleration = 15.0;
  r._observation_sequence_number = seq;
  return r;
}

// A rectangular prism from x0 to x1 ahead of the robot, |y| <= half_width,
// 0.4-1.2 m high (Servi-Q's blind-spot and inter-sensor prisms).
FootprintClearingPrism::Config MakePrismConfig(double x0, double x1,
                                               double half_width) {
  FootprintClearingPrism::Config c;
  c.enable = true;
  c.publish_visualization = false;
  c.min_z = 0.4;
  c.max_z = 1.2;
  c.footprint_points = {FootprintClearingPrism::Point(x0, half_width),
                        FootprintClearingPrism::Point(x1, half_width),
                        FootprintClearingPrism::Point(x1, -half_width),
                        FootprintClearingPrism::Point(x0, -half_width)};
  return c;
}

std::unique_ptr<SpatioTemporalVoxelGrid> MakeServiQGrid(double voxel_size,
                                                        ros::NodeHandle& nh) {
  InterSensorDecayPrism::Config inter_sensor;
  inter_sensor.prism_config = MakePrismConfig(0.8, 2.0, 0.8);
  inter_sensor.decay_acceleration_factor = 5.0;
  auto grid = std::make_unique<SpatioTemporalVoxelGrid>(
      static_cast<float>(voxel_size), 0.0, volume_grid::LINEAR, kVoxelDecay,
      /*pub_voxels=*/true, test_utils::MakeMockFootprintFrustum(),
      InterSensorDecayPrism::Create(inter_sensor, nh),
      FootprintClearingPrism::Create(MakePrismConfig(0.0, 0.32, 0.2), nh),
      test_utils::MakeMockDynamicObstacleTracker(),
      test_utils::MakeMockRobotMotionTracker());
  grid->SetRobotPose(0, 0, 0);
  return grid;
}

// Servi-Q geometry: pcl_post_processing keeps xy range 0.25-1.5 m within
// +-0.9505 rad, the down camera arrives at 3.5 cm (max voxel) and the up
// camera at 5 cm (centroid voxel); the grid holds the strip swept over the
// 15 s decay window at ~0.5 m/s.
struct Geometry {
  bool servi_q;
  double spacing_low;   // z < kSplitZ (down camera)
  double spacing_high;  // z >= kSplitZ (up camera)
};
constexpr double kSplitZ = 0.6;

std::vector<Point> MakeMixedScene(const Geometry& g) {
  std::vector<Point> out;
  for (const Point& p : MakeScene(g.spacing_low))
    if (p.z < kSplitZ) out.push_back(p);
  for (const Point& p : MakeScene(g.spacing_high))
    if (p.z >= kSplitZ) out.push_back(p);
  return out;
}

// What the grid already holds when timing starts.
std::vector<Point> HistoryInDecayWindow(const std::vector<Point>& scene,
                                        const Geometry& g) {
  if (!g.servi_q) return scene;
  std::vector<Point> history;
  for (const Point& p : scene)
    if (p.x > -7.5 && p.x < 1.5 && std::fabs(p.y) <= 1.5) history.push_back(p);
  return history;
}

struct CameraClouds {
  std::vector<Point> up;
  std::vector<Point> down;
};

CameraClouds CameraCloudsPerCycle(const std::vector<Point>& scene,
                                  const Geometry& g) {
  CameraClouds clouds;
  if (g.servi_q) {
    for (const Point& p : CameraView(scene, ViewWedge{0.0, 0.9505, 0.25, 1.5}))
      (p.z < kSplitZ ? clouds.down : clouds.up).push_back(p);
  } else {
    clouds.up = CameraView(scene, ViewWedge{0.0, 0.51});
    clouds.down = CameraView(scene, ViewWedge{0.0, 0.75});
  }
  return clouds;
}

struct CycleTimes {
  double clear_ms = 0, mark_ms = 0, flatten_ms = 0, publish_ms = 0;
  // Voxels that survived the clear pass; ClearFrustums iterates all of them,
  // so this is what the per-update cost scales with. With pub_voxels on, the
  // published cloud holds one point per surviving voxel.
  size_t active_voxels = 0;
  // Distinct (x, y) columns in the flattened costmap map.
  size_t costmap_columns = 0;
};

double MsBetween(std::chrono::steady_clock::time_point from,
                 std::chrono::steady_clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

// One STVL update in the order updateBounds runs it.
CycleTimes TimeOneCycle(
    SpatioTemporalVoxelGrid& grid,
    const std::vector<observation::MeasurementReading>& readings) {
  std::unordered_set<occupany_cell> cleared;
  std::vector<DynamicObstacleReading> dynamic_obstacles;
  std::vector<RobotMotionReading> robot_motion;
  const auto t0 = std::chrono::steady_clock::now();
  grid.ClearFrustums(readings, cleared, dynamic_obstacles, robot_motion);
  const auto t1 = std::chrono::steady_clock::now();
  grid.Mark(readings);
  const auto t2 = std::chrono::steady_clock::now();
  // Mirror UpdateROSCostmap's walk over the flattened map.
  volatile size_t sink = 0;
  for (const auto& kv : *grid.GetFlattenedCostmap()) sink += kv.second;
  const auto t3 = std::chrono::steady_clock::now();
  sensor_msgs::PointCloud2::Ptr pc2(new sensor_msgs::PointCloud2());
  grid.GetAllVoxelsPointCloud(pc2);
  const auto t4 = std::chrono::steady_clock::now();
  return {MsBetween(t0, t1), MsBetween(t1, t2),
          MsBetween(t2, t3), MsBetween(t3, t4),
          pc2->width,        grid.GetFlattenedCostmap()->size()};
}

// Average per-stage times over kTimedCycles after kWarmupCycles; voxel counts
// come from the last cycle.
CycleTimes TimeUpdateCycles(SpatioTemporalVoxelGrid& grid,
                            ros::NodeHandle& frustum_nh,
                            const CameraClouds& clouds) {
  CycleTimes total;
  uint64_t seq = 2;  // the seed observation is 1
  for (int i = 0; i < kWarmupCycles + kTimedCycles; ++i, ++seq) {
    const std::vector<observation::MeasurementReading> readings = {
        MakeReading(frustum_nh, "astra_up", clouds.up, 1.0, -0.3, seq),
        MakeReading(frustum_nh, "gemini_down", clouds.down, 0.35, 0.5, seq),
        MakeReading(frustum_nh, "gemini_cliff", {}, 0.35, 0.5, seq)};
    const CycleTimes cycle = TimeOneCycle(grid, readings);
    total.active_voxels = cycle.active_voxels;
    total.costmap_columns = cycle.costmap_columns;
    if (i < kWarmupCycles) continue;
    total.clear_ms += cycle.clear_ms / kTimedCycles;
    total.mark_ms += cycle.mark_ms / kTimedCycles;
    total.flatten_ms += cycle.flatten_ms / kTimedCycles;
    total.publish_ms += cycle.publish_ms / kTimedCycles;
  }
  return total;
}

struct Result {
  CycleTimes times;
  size_t history_points;
  size_t camera_points_per_cycle;
};

Result RunCase(double voxel_size, const Geometry& g) {
  ros::NodeHandle nh;
  ros::NodeHandle frustum_nh("/bench/frustum");
  openvdb::initialize();
  const std::unique_ptr<SpatioTemporalVoxelGrid> grid =
      MakeServiQGrid(voxel_size, nh);

  const std::vector<Point> scene = MakeMixedScene(g);
  const std::vector<Point> history = HistoryInDecayWindow(scene, g);
  std::vector<observation::MeasurementReading> seed = {
      MakeReading(frustum_nh, "seed", history, 0.5, 0, 1)};
  seed.back()._obstacle_range_in_m = 100.0;
  grid->Mark(seed);

  const CameraClouds clouds = CameraCloudsPerCycle(scene, g);
  return {TimeUpdateCycles(*grid, frustum_nh, clouds), history.size(),
          clouds.up.size() + clouds.down.size()};
}

struct BenchCase {
  const char* label;
  double voxel_size;
  Geometry geometry;
};

void PrintTable(const char* title, const std::vector<BenchCase>& cases) {
  // STVL runs in the costmap update thread (local costmap update_frequency).
  constexpr double kUpdateHz = 20.0;
  // The voxel cloud publishes at 5 Hz (publish_voxel_map_period 0.2 s).
  constexpr double kPublishHz = 5.0;
  std::printf("\n== %s ==\n%-34s %9s %9s %9s %9s %8s %8s %8s %8s %8s %7s\n",
              title, "case", "voxels", "columns", "hist_pt", "cam_pt/c",
              "clr_ms", "mark_ms", "flat_ms", "pub_ms", "cyc_ms", "cpu@20");
  for (const BenchCase& c : cases) {
    const Result r = RunCase(c.voxel_size, c.geometry);
    const CycleTimes& t = r.times;
    const double cycle_ms = t.clear_ms + t.mark_ms + t.flatten_ms +
                            t.publish_ms * kPublishHz / kUpdateHz;
    std::printf(
        "%-34s %9zu %9zu %9zu %9zu %8.2f %8.2f %8.2f %8.2f %8.2f %6.1f%%\n",
        c.label, t.active_voxels, t.costmap_columns, r.history_points,
        r.camera_points_per_cycle, t.clear_ms, t.mark_ms, t.flatten_ms,
        t.publish_ms, cycle_ms, cycle_ms * kUpdateHz / 10.0);
  }
}

TEST(BenchVoxelSize, Table) {
  const Geometry servi{true, 0.035, 0.05};
  const Geometry servi_2cm_input{true, 0.02, 0.02};
  PrintTable("Servi-Q geometry (1.5 m post-processing range, 15 s swath)",
             {{"today: 4cm voxel / 3.5+5cm input", 0.04, servi},
              {"2cm voxel / 3.5+5cm input", 0.02, servi},
              {"2cm voxel / 2cm input", 0.02, servi_2cm_input}});
  const Geometry room{false, 0.05, 0.05};
  const Geometry room_2cm{false, 0.02, 0.02};
  PrintTable("Worst case: whole 12x8 m room in the grid, 4 m camera range",
             {{"4cm voxel / 5cm input", 0.04, room},
              {"2cm voxel / 5cm input", 0.02, room},
              {"2cm voxel / 2cm input", 0.02, room_2cm}});
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bench_voxel_size");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
