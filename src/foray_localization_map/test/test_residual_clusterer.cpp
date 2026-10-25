// Copyright 2026 ZQU-Foray

#include "foray/localization_map/residual_clusterer.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using foray::localization_map::PointCloud;
using foray::localization_map::ResidualClusterer;
using foray::localization_map::ResidualClustererConfig;

PointCloud make_plane(int side, double spacing) {
    PointCloud cloud;
    for (int iy = 0; iy < side; ++iy) {
        for (int ix = 0; ix < side; ++ix) {
            cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                      static_cast<float>(iy * spacing), 0.0F);
        }
    }
    return cloud;
}

TEST(ResidualClustererTest, FindsDynamicClusterAboveMap) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 20; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.01F * static_cast<float>(i));
    }

    const ResidualClusterer clusterer;
    const auto clusters = clusterer.cluster(source, target);
    ASSERT_EQ(clusters.size(), 1U);
    EXPECT_EQ(clusters[0].point_count, 20);
    EXPECT_NEAR(clusters[0].centroid.z(), 0.6, 0.05);
    EXPECT_GT(clusters[0].confidence, 0.5);
}

TEST(ResidualClustererTest, DropsSmallClusters) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    source.points.emplace_back(5.0F, 5.0F, 5.0F);
    source.points.emplace_back(5.0F, 5.0F, 5.05F);

    const ResidualClusterer clusterer;
    EXPECT_TRUE(clusterer.cluster(source, target).empty());
}

TEST(ResidualClustererTest, SplitsSeparatedClusters) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 10; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.01F * static_cast<float>(i));
        source.points.emplace_back(3.0F, 3.0F, 0.5F + 0.01F * static_cast<float>(i));
    }

    const ResidualClusterer clusterer;
    const auto clusters = clusterer.cluster(source, target);
    ASSERT_EQ(clusters.size(), 2U);
}

TEST(ResidualClustererTest, RespectsMaxClusterSize) {
    const PointCloud target = make_plane(21, 0.1);
    PointCloud source = target;
    for (int i = 0; i < 60; ++i) {
        source.points.emplace_back(0.5F, 0.5F, 0.5F + 0.005F * static_cast<float>(i));
    }

    ResidualClustererConfig config;
    config.max_cluster_size = 50;
    const ResidualClusterer clusterer(config);
    EXPECT_TRUE(clusterer.cluster(source, target).empty());
}

} // namespace
