/*********************************************************************
 *
 * Software License Agreement (LGPL 2.1)
 *
 *  Copyright (c) 2025-2026, Bear Robotics, Inc.
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
 *
 * Author: Seung-Hun (Hoon) Han (seunghun.han@bearrobotics.ai)
 *********************************************************************/

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "test/test_utils.h"

namespace {

using spatio_temporal_voxel_layer::test_utils::kVoxelSize;
using spatio_temporal_voxel_layer::test_utils::MakePointCloud;
using spatio_temporal_voxel_layer::test_utils::MakeTestGrid;

geometry_msgs::Point MakePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

class MarkSnappingTest : public ::testing::Test {
 protected:
  void SetUp() override { grid_ = MakeTestGrid(); }

  void MarkPoint(const geometry_msgs::Point& point) {
    observation::MeasurementReading reading;
    reading._cloud =
        boost::make_shared<sensor_msgs::PointCloud2>(MakePointCloud({point}));
    reading._origin = MakePoint(0.0, 0.0, 0.0);
    reading._marking = true;
    reading._clearing = false;
    reading._obstacle_range_in_m = 100.0;
    grid_->Mark({reading});
  }

  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> grid_;
};

// A negative coordinate must snap down to the voxel that contains it (floor),
// not toward zero. With kVoxelSize 0.05, z=-0.12 falls in voxel index -3, whose
// center is -0.125. The snap is per-axis: a positive Y must not suppress it.
TEST_F(MarkSnappingTest, NegativeZSnapsDownRegardlessOfY) {
  MarkPoint(MakePoint(0.22, 0.22, -0.12));

  std::vector<openvdb::Vec3d> voxels =
      grid_->GetVoxelsAtXY(0.22, 0.22, kVoxelSize);

  ASSERT_EQ(voxels.size(), 1u);
  EXPECT_NEAR(voxels[0].z(), -0.125, 1e-4);
}

// A positive Z must not be dragged down a voxel by a negative Y. z=0.12 falls
// in voxel index 2, whose center is 0.125.
TEST_F(MarkSnappingTest, PositiveZNotSnappedByNegativeY) {
  MarkPoint(MakePoint(0.22, -0.22, 0.12));

  std::vector<openvdb::Vec3d> voxels =
      grid_->GetVoxelsAtXY(0.22, -0.22, kVoxelSize);

  ASSERT_EQ(voxels.size(), 1u);
  EXPECT_NEAR(voxels[0].z(), 0.125, 1e-4);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mark_snapping");
  return RUN_ALL_TESTS();
}
