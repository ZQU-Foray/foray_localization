// Copyright 2026 ZQU-Foray

#include "foray/localization_map/point_cloud.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using foray::localization_map::PointCloud;
using foray::localization_map::transform_points;
using foray::localization_map::voxel_downsample;

TEST(PointCloudTest, EmptyCloudStaysEmpty) {
    const PointCloud cloud;
    EXPECT_TRUE(cloud.empty());
    EXPECT_EQ(voxel_downsample(cloud, 0.5).size(), 0U);
}

TEST(PointCloudTest, VoxelDownsampleCollapsesSingleVoxelToMean) {
    PointCloud cloud;
    cloud.points.emplace_back(0.1F, 0.1F, 0.1F);
    cloud.points.emplace_back(0.9F, 0.9F, 0.9F);
    cloud.points.emplace_back(0.5F, 0.5F, 0.5F);

    const PointCloud downsampled = voxel_downsample(cloud, 1.0);
    ASSERT_EQ(downsampled.size(), 1U);
    EXPECT_NEAR(downsampled.points[0].x(), 0.5F, 1e-5F);
    EXPECT_NEAR(downsampled.points[0].y(), 0.5F, 1e-5F);
    EXPECT_NEAR(downsampled.points[0].z(), 0.5F, 1e-5F);
}

TEST(PointCloudTest, VoxelDownsampleKeepsSeparateVoxels) {
    PointCloud cloud;
    cloud.points.emplace_back(0.1F, 0.1F, 0.1F);
    cloud.points.emplace_back(1.1F, 0.1F, 0.1F);

    EXPECT_EQ(voxel_downsample(cloud, 0.5).size(), 2U);
}

TEST(PointCloudTest, TransformAppliesRigidMotion) {
    PointCloud cloud;
    cloud.points.emplace_back(1.0F, 0.0F, 0.0F);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translate(Eigen::Vector3d(0.0, 2.0, 0.0));

    const PointCloud transformed = transform_points(cloud, transform);
    ASSERT_EQ(transformed.size(), 1U);
    EXPECT_NEAR(transformed.points[0].x(), 1.0F, 1e-5F);
    EXPECT_NEAR(transformed.points[0].y(), 2.0F, 1e-5F);
    EXPECT_NEAR(transformed.points[0].z(), 0.0F, 1e-5F);
}

} // namespace
