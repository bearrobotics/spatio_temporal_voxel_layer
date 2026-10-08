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

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace {

using spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer;

// The ns of the rosparam block in test_layer_voxel_class_params.test.
constexpr char kParamNamespace[] = "/stvl_layer_params_test";
constexpr char kGlobalFrame[] = "map";
constexpr unsigned int kMapCells = 20;
constexpr double kResolution = 0.05;
constexpr double kMapOrigin = -0.5;

unsigned int CountCellsWithCost(const costmap_2d::Costmap2D& costmap,
                                unsigned char value) {
  unsigned int count = 0;
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
      if (costmap.getCost(i, j) == value) {
        ++count;
      }
    }
  }
  return count;
}

class LayerVoxelClassParamsTest : public ::testing::Test {
 protected:
  LayerVoxelClassParamsTest()
      : parent_nh_(kParamNamespace),
        layered_costmap_(parent_nh_, kGlobalFrame, /*rolling_window=*/false,
                         /*track_unknown=*/false) {
    layered_costmap_.resizeMap(kMapCells, kMapCells, kResolution, kMapOrigin,
                               kMapOrigin);
  }

  // layer_name selects a config block in test_layer_voxel_class_params.yaml.
  boost::shared_ptr<SpatioTemporalVoxelLayer> InitializeLayer(
      const std::string& layer_name) {
    auto layer = boost::make_shared<SpatioTemporalVoxelLayer>();
    layered_costmap_.addPlugin(layer);
    layer->initialize(&layered_costmap_, layer_name, parent_nh_,
                      /*tf=*/nullptr);
    return layer;
  }

  // A missing required param shuts the node down instead of throwing.
  void ExpectInitializedAndEmpty(
      const boost::shared_ptr<SpatioTemporalVoxelLayer>& layer) {
    ASSERT_TRUE(ros::ok()) << "a required layer param is missing";

    layered_costmap_.updateMap(0.0, 0.0, 0.0);

    EXPECT_TRUE(layer->isCurrent());
    EXPECT_EQ(CountCellsWithCost(*layered_costmap_.getCostmap(),
                                 costmap_2d::LETHAL_OBSTACLE),
              0u);
  }

  ros::NodeHandle parent_nh_;
  costmap_2d::LayeredCostmap layered_costmap_;
};

TEST_F(LayerVoxelClassParamsTest, ClassifiedSourcesInitializeAndStartEmpty) {
  ExpectInitializedAndEmpty(InitializeLayer("classified"));
}

TEST_F(LayerVoxelClassParamsTest, SourcesDefaultToGenericWithoutVoxelClasses) {
  ExpectInitializedAndEmpty(InitializeLayer("generic_only"));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_layer_voxel_class_params");
  return RUN_ALL_TESTS();
}
