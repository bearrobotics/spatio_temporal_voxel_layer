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
 * Author: Vincent Benenati (vincent.benenati@bearrobotics.ai)
 *********************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "spatio_temporal_voxel_layer/filter_factory.h"
#include "spatio_temporal_voxel_layer/noise_filter.h"
#include "spatio_temporal_voxel_layer/passthrough_filter.h"

namespace spatio_temporal_voxel_layer {

// ============================================================================
// FilterFactory Tests
// ============================================================================

TEST(FilterFactoryTest, CreateNoiseFilterWithValidParams) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_NE(filter, nullptr);
}

TEST(FilterFactoryTest, CreateFilterWithTypeNone) {
  ros::NodeHandle nh("/filter_factory_test/type_none");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateFilterWithTypeVoxel) {
  ros::NodeHandle nh("/filter_factory_test/type_voxel");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateFilterWithTypePassthrough) {
  ros::NodeHandle nh("/filter_factory_test/type_passthrough");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_NE(filter, nullptr);
}

TEST(FilterFactoryTest, CreateFilterWithUnknownType) {
  ros::NodeHandle nh("/filter_factory_test/type_unknown");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateFilterWithEmptyType) {
  ros::NodeHandle nh("/filter_factory_test/type_empty");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMissingMinHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_min_height");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMissingMaxHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_max_height");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMissingMeanK) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_mean_k");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMissingStdDev) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_std_dev");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMissingVoxelMin) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_voxel_min");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterNegativeMinHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_negative_min_height");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterMaxLessThanMin) {
  ros::NodeHandle nh("/filter_factory_test/noise_max_less_than_min");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterZeroMeanK) {
  ros::NodeHandle nh("/filter_factory_test/noise_zero_mean_k");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterNegativeStdDev) {
  ros::NodeHandle nh("/filter_factory_test/noise_negative_std_dev");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(FilterFactoryTest, CreateNoiseFilterZeroVoxelMin) {
  ros::NodeHandle nh("/filter_factory_test/noise_zero_voxel_min");
  auto filter = FilterFactory::CreateFilter(nh);
  EXPECT_EQ(filter, nullptr);
}

// ============================================================================
// NoiseFilter::LoadConfig Tests
// ============================================================================

TEST(NoiseFilterTest, LoadConfigWithValidParams) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  auto config = NoiseFilter::Config::Load(nh);
  ASSERT_TRUE(config.has_value());
  EXPECT_FLOAT_EQ(config->min_obstacle_height, 0.1f);
  EXPECT_FLOAT_EQ(config->max_obstacle_height, 2.0f);
  EXPECT_EQ(config->mean_k, 10);
  EXPECT_FLOAT_EQ(config->std_dev_mul_thresh, 1.0f);
  EXPECT_EQ(config->voxel_min_points, 1);
}

TEST(NoiseFilterTest, LoadConfigMissingMinObstacleHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_min_height");
  auto config = NoiseFilter::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(NoiseFilterTest, LoadConfigMissingMaxObstacleHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_max_height");
  auto config = NoiseFilter::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(NoiseFilterTest, LoadConfigMissingMeanK) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_mean_k");
  auto config = NoiseFilter::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(NoiseFilterTest, LoadConfigMissingStdDevMulThresh) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_std_dev");
  auto config = NoiseFilter::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

TEST(NoiseFilterTest, LoadConfigMissingVoxelMinPoints) {
  ros::NodeHandle nh("/filter_factory_test/noise_missing_voxel_min");
  auto config = NoiseFilter::Config::Load(nh);
  EXPECT_FALSE(config.has_value());
}

// ============================================================================
// NoiseFilter::Create Tests (Config Validation)
// ============================================================================

TEST(NoiseFilterTest, CreateWithValidConfig) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_NE(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithNegativeMinObstacleHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = -0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithMaxLessThanMin) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 2.0f;
  config.max_obstacle_height = 0.1f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithZeroMeanK) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 0;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithNegativeMeanK) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = -1;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithNegativeStdDevMulThresh) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = -1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithZeroVoxelMinPoints) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 0;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

TEST(NoiseFilterTest, CreateWithNegativeVoxelMinPoints) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = -1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_EQ(filter, nullptr);
}

// ============================================================================
// NoiseFilter::Filter Tests
// ============================================================================

namespace {

pcl::PCLPointCloud2::Ptr CreatePointCloud(
    const std::vector<pcl::PointXYZ>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pt : points) {
    cloud_xyz->push_back(pt);
  }
  cloud_xyz->width = cloud_xyz->size();
  cloud_xyz->height = 1;
  cloud_xyz->is_dense = true;

  pcl::PCLPointCloud2::Ptr cloud_pcl2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*cloud_xyz, *cloud_pcl2);
  return cloud_pcl2;
}

std::vector<pcl::PointXYZ> ExtractPoints(
    const pcl::PCLPointCloud2::Ptr& cloud_pcl2) {
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::fromPCLPointCloud2(*cloud_pcl2, cloud_xyz);
  return std::vector<pcl::PointXYZ>(cloud_xyz.begin(), cloud_xyz.end());
}

}  // namespace

TEST(NoiseFilterTest, FilterEmptyCloud) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  auto cloud = CreatePointCloud({});
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);
  EXPECT_EQ(points.size(), 0);
}

TEST(NoiseFilterTest, FilterRemovesPointsBelowMinHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.5f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  // Add points below min height (should be removed by PassThrough)
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 0.2f));
  input_points.push_back(pcl::PointXYZ(3.0f, 3.0f, 0.3f));
  // Add multiple points in valid range (need enough for
  // StatisticalOutlierRemoval)
  for (int i = 0; i < 10; ++i) {
    input_points.push_back(pcl::PointXYZ(2.0f + i * 0.1f, 2.0f, 1.0f));
  }

  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);

  // All points should be in valid height range
  for (const auto& pt : points) {
    EXPECT_GE(pt.z, 0.5f);
    EXPECT_LE(pt.z, 2.0f);
  }
  // Should have removed the 2 points below min height
  EXPECT_LT(points.size(), input_points.size());
}

TEST(NoiseFilterTest, FilterRemovesPointsAboveMaxHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 1.5f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  // Add points above max height (should be removed by PassThrough)
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 2.0f));
  input_points.push_back(pcl::PointXYZ(3.0f, 3.0f, 3.0f));
  // Add multiple points in valid range (need enough for
  // StatisticalOutlierRemoval)
  for (int i = 0; i < 10; ++i) {
    input_points.push_back(pcl::PointXYZ(2.0f + i * 0.1f, 2.0f, 1.0f));
  }

  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);

  // All points should be in valid height range
  for (const auto& pt : points) {
    EXPECT_GE(pt.z, 0.1f);
    EXPECT_LE(pt.z, 1.5f);
  }
  // Should have removed the 2 points above max height
  EXPECT_LT(points.size(), input_points.size());
}

TEST(NoiseFilterTest, FilterKeepsPointsInHeightRange) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.5f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 0.6f));
  input_points.push_back(pcl::PointXYZ(2.0f, 2.0f, 1.0f));
  input_points.push_back(pcl::PointXYZ(3.0f, 3.0f, 1.9f));

  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);

  EXPECT_EQ(points.size(), 3);
}

TEST(NoiseFilterTest, FilterIsReentrant) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.5f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));
  input_points.push_back(pcl::PointXYZ(2.0f, 2.0f, 1.5f));

  auto cloud1 = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud1);
  auto points1 = ExtractPoints(cloud1);
  EXPECT_EQ(points1.size(), 2);

  auto cloud2 = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud2);
  auto points2 = ExtractPoints(cloud2);
  EXPECT_EQ(points2.size(), 2);

  filter->ApplyFilter(cloud1);
  auto points3 = ExtractPoints(cloud1);
  EXPECT_EQ(points3.size(), 2);
}

TEST(NoiseFilterTest, FilterWithZeroWidthCloud) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  cloud->width = 0;
  cloud->height = 1;
  filter->ApplyFilter(cloud);
  EXPECT_EQ(cloud->width, 0);
}

TEST(NoiseFilterTest, FilterWithZeroHeightCloud) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  cloud->width = 1;
  cloud->height = 0;
  filter->ApplyFilter(cloud);
  EXPECT_EQ(cloud->height, 0);
}

TEST(NoiseFilterTest, FilterWithEmptyDataCloud) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.1f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  cloud->width = 1;
  cloud->height = 1;
  cloud->data.clear();
  filter->ApplyFilter(cloud);
  EXPECT_TRUE(cloud->data.empty());
}

TEST(NoiseFilterTest, FilterAtBoundaryMinHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.5f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 0.5f));

  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);

  EXPECT_EQ(points.size(), 1);
}

TEST(NoiseFilterTest, FilterAtBoundaryMaxHeight) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 0.5f;
  config.max_obstacle_height = 2.0f;
  config.mean_k = 1;
  config.std_dev_mul_thresh = 100.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points;
  input_points.push_back(pcl::PointXYZ(1.0f, 1.0f, 2.0f));

  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);

  EXPECT_EQ(points.size(), 1);
}

// ============================================================================
// PassthroughFilter Tests
// ============================================================================

TEST(PassthroughFilterTest, ApplyFilterLeavesNonEmptyCloudUnchanged) {
  std::vector<pcl::PointXYZ> input_points = {
      pcl::PointXYZ(1.0f, 1.0f, -10.0f),
      pcl::PointXYZ(2.0f, 2.0f, 0.0f),
      pcl::PointXYZ(3.0f, 3.0f, 10.0f),
  };
  auto cloud = CreatePointCloud(input_points);
  auto data_before = cloud->data;
  uint32_t width_before = cloud->width;

  PassthroughFilter filter;
  filter.ApplyFilter(cloud);

  EXPECT_EQ(cloud->width, width_before);
  EXPECT_EQ(cloud->data, data_before);
  auto points = ExtractPoints(cloud);
  ASSERT_EQ(points.size(), input_points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(points[i].x, input_points[i].x);
    EXPECT_FLOAT_EQ(points[i].y, input_points[i].y);
    EXPECT_FLOAT_EQ(points[i].z, input_points[i].z);
  }
}

TEST(PassthroughFilterTest, ApplyFilterOnEmptyCloudIsSafe) {
  PassthroughFilter filter;
  auto cloud = CreatePointCloud({});
  filter.ApplyFilter(cloud);
  EXPECT_EQ(ExtractPoints(cloud).size(), 0);
}

TEST(FilterFactoryTest, PassthroughFilterIsNoOp) {
  ros::NodeHandle nh("/filter_factory_test/type_passthrough");
  auto filter = FilterFactory::CreateFilter(nh);
  ASSERT_NE(filter, nullptr);

  std::vector<pcl::PointXYZ> input_points = {
      pcl::PointXYZ(0.0f, 0.0f, -5.0f),
      pcl::PointXYZ(1.0f, 1.0f, 1.0f),
      pcl::PointXYZ(2.0f, 2.0f, 5.0f),
  };
  auto cloud = CreatePointCloud(input_points);
  filter->ApplyFilter(cloud);
  auto points = ExtractPoints(cloud);
  EXPECT_EQ(points.size(), input_points.size());
}

TEST(NoiseFilterTest, CreateWithZeroHeightRange) {
  ros::NodeHandle nh("/filter_factory_test/noise_valid");
  NoiseFilter::Config config;
  config.min_obstacle_height = 1.0f;
  config.max_obstacle_height = 1.0f;
  config.mean_k = 10;
  config.std_dev_mul_thresh = 1.0f;
  config.voxel_min_points = 1;
  auto filter = NoiseFilter::Create(config, nh);
  EXPECT_NE(filter, nullptr);
}

}  // namespace spatio_temporal_voxel_layer

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_filter_factory");
  return RUN_ALL_TESTS();
}
