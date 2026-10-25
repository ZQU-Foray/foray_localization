// Copyright 2026 ZQU-Foray

#include "foray/localization_map/icp_aligner.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using foray::localization_map::AlignOptions;
using foray::localization_map::IcpAligner;
using foray::localization_map::PointCloud;

PointCloud make_grid(int side, double spacing) {
    PointCloud cloud;
    for (int iy = 0; iy < side; ++iy) {
        for (int ix = 0; ix < side; ++ix) {
            cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                      static_cast<float>(iy * spacing), 0.0F);
        }
    }
    return cloud;
}

TEST(IcpAlignerTest, RecoversKnownTransform) {
    const PointCloud target = make_grid(20, 0.1);

    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(0.05, -0.03, 0.0);
    Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    AlignOptions options;
    options.max_iterations = 40;
    options.max_correspondence_distance_m = 0.3;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    ASSERT_TRUE(result.converged) << result.error;
    EXPECT_LT((result.transform.translation() - truth.translation()).norm(), 0.02);
    const double angle =
        Eigen::AngleAxisd(result.transform.linear() * truth.linear().transpose()).angle();
    EXPECT_LT(std::abs(angle), 0.02);
    EXPECT_GT(result.inlier_ratio, 0.95);
}

TEST(IcpAlignerTest, FailsOnDisjointClouds) {
    PointCloud source = make_grid(5, 0.1);
    PointCloud target = make_grid(5, 0.1);
    for (auto& point : target.points) {
        point.x() += 100.0F;
    }

    AlignOptions options;
    options.max_correspondence_distance_m = 0.2;
    IcpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_FALSE(result.error.empty());
}

TEST(IcpAlignerTest, RejectsEmptyInput) {
    const PointCloud empty;
    IcpAligner aligner;
    const auto result = aligner.align(empty, empty, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
}

} // namespace
