// Copyright 2026 ZQU-Foray
//
// PCL GICP 后端的契约测试。仅在 FORAY_WITH_PCL 时构建（CMakeLists 里条件注册）。
//
// 注意点云形状：这里用三维格点，不用 test_icp 里的平面网格。GICP 按邻域协方差
// 加权，纯共面点云在法向上协方差退化，本身就是病态输入，用它测 GICP 只会测出噪声。

#include "foray/localization_map/pcl_gicp_aligner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace {

using foray::localization_map::AlignOptions;
using foray::localization_map::PclGicpAligner;
using foray::localization_map::PointCloud;

PointCloud make_cube(int side, double spacing) {
    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(side * side * side));
    for (int iz = 0; iz < side; ++iz) {
        for (int iy = 0; iy < side; ++iy) {
            for (int ix = 0; ix < side; ++ix) {
                cloud.points.emplace_back(static_cast<float>(ix * spacing),
                                          static_cast<float>(iy * spacing),
                                          static_cast<float>(iz * spacing));
            }
        }
    }
    return cloud;
}

AlignOptions make_options() {
    AlignOptions options;
    options.max_iterations = 50;
    options.max_correspondence_distance_m = 0.5;
    return options;
}

TEST(PclGicpAlignerTest, RecoversKnownTransform) {
    // spacing 0.3 让点间距远大于初始错位（< 0.06 m），最近邻对应关系无歧义。
    const PointCloud target = make_cube(5, 0.3);

    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(0.05, -0.03, 0.01);
    const Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    source.points.reserve(target.points.size());
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    PclGicpAligner aligner(make_options());
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    ASSERT_TRUE(result.converged) << result.error;
    EXPECT_LT((result.transform.translation() - truth.translation()).norm(), 0.02);
    const double angle =
        Eigen::AngleAxisd(result.transform.linear() * truth.linear().transpose()).angle();
    EXPECT_LT(std::abs(angle), 0.02);
    EXPECT_GT(result.inlier_ratio, 0.9);
    EXPECT_GT(result.iterations, 0);
}

TEST(PclGicpAlignerTest, RejectsEmptyInput) {
    const PointCloud empty;
    PclGicpAligner aligner;
    const auto result = aligner.align(empty, empty, Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_FALSE(result.error.empty());
    EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.0);
}

TEST(PclGicpAlignerTest, FailsOnDisjointClouds) {
    const PointCloud source = make_cube(4, 0.2);
    PointCloud target = make_cube(4, 0.2);
    for (auto& point : target.points) {
        point.x() += 100.0F;
    }

    AlignOptions options;
    options.max_correspondence_distance_m = 0.2;
    PclGicpAligner aligner(options);
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.0);
    EXPECT_FALSE(result.error.empty());
}

TEST(PclGicpAlignerTest, MetricsAreConsistentWithIcpSemantics) {
    // 守住共享评估口径的语义契约（decision.md D21）：fitness / inlier_ratio 都是
    // 0..1 且越大越好。PCL 自带的 getFitnessScore() 是均方误差（越小越好），若有人
    // 图省事直接把它填进 AlignResult，这两条断言会立刻失败。
    const PointCloud target = make_cube(5, 0.3);
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitY()).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(-0.04, 0.02, 0.03);
    const Eigen::Isometry3d inverse = truth.inverse();

    PointCloud source;
    source.points.reserve(target.points.size());
    for (const auto& point : target.points) {
        source.points.push_back((inverse * point.cast<double>()).cast<float>());
    }

    PclGicpAligner aligner(make_options());
    const auto result = aligner.align(source, target, Eigen::Isometry3d::Identity());

    EXPECT_GT(result.inlier_ratio, 0.0);
    EXPECT_LE(result.inlier_ratio, 1.0);
    EXPECT_GT(result.fitness, 0.0);
    EXPECT_LE(result.fitness, 1.0);
    EXPECT_GE(result.mean_residual_m, 0.0);
    EXPECT_LE(result.mean_residual_m, aligner.options().max_correspondence_distance_m);
}

} // namespace
