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

#include <string>

#include "bear_msgs/ClearClassVoxels.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace {

// Only the rejection paths run here: they return before the callback touches
// the voxel grid, so an uninitialized layer is enough. Clearing a class is
// covered at the grid level by test_class_publish_and_clear.
bear_msgs::ClearClassVoxels::Response CallClearClassVoxels(
    const std::string& class_name) {
  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer layer;
  bear_msgs::ClearClassVoxels::Request req;
  bear_msgs::ClearClassVoxels::Response resp;
  req.class_name = class_name;
  EXPECT_TRUE(layer.ClearClassVoxelsCallback(req, resp));
  return resp;
}

TEST(ClearClassVoxelsServiceTest, UnknownClassNameIsRejected) {
  const bear_msgs::ClearClassVoxels::Response resp =
      CallClearClassVoxels("not_a_class");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_NE(resp.message.find("not_a_class"), std::string::npos)
      << "a rejection must name the class the caller sent";
}

TEST(ClearClassVoxelsServiceTest, KnownClassBeforeInitializeIsRejected) {
  const bear_msgs::ClearClassVoxels::Response resp =
      CallClearClassVoxels("cliff");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_FALSE(resp.message.empty());
}

TEST(ClearClassVoxelsServiceTest, EmptyClassNameIsRejected) {
  const bear_msgs::ClearClassVoxels::Response resp = CallClearClassVoxels("");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_FALSE(resp.message.empty());
}

TEST(ClearClassVoxelsServiceTest, GenericIsRejected) {
  const bear_msgs::ClearClassVoxels::Response resp =
      CallClearClassVoxels("generic");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
  EXPECT_FALSE(resp.message.empty());
}

TEST(ClearClassVoxelsServiceTest, CapitalizedClassNameIsRejected) {
  const bear_msgs::ClearClassVoxels::Response resp =
      CallClearClassVoxels("Cliff");

  EXPECT_FALSE(resp.success);
  EXPECT_EQ(resp.cleared_voxels, 0u);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_clear_class_voxels_service");
  return RUN_ALL_TESTS();
}
