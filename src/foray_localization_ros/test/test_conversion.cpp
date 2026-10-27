// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/conversion.hpp"

#include <gtest/gtest.h>

namespace {

using foray::localization::HealthState;
using foray::localization::HealthStatus;
using foray::localization::MapManifest;
using foray::localization::Pose3D;
using foray::localization::PoseSource;
using foray::localization_map::ResidualCluster;
using foray::localization_ros::to_msg_map_meta;
using foray::localization_ros::to_msg_obstacles;
using foray::localization_ros::to_msg_pose;
using foray::localization_ros::to_msg_state;
using foray::localization_ros::to_msg_time;

TEST(ConversionTest, StateCarriesHealthAndPose) {
    HealthStatus health;
    health.state = HealthState::kDegraded;
    health.confidence = 0.42;
    health.age_sec = 0.25;
    health.reason = "残差偏大";

    Pose3D pose;
    pose.parent_frame = "map";
    pose.child_frame = "base_footprint";
    pose.transform = Eigen::Isometry3d::Identity();
    pose.transform.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);

    const auto message = to_msg_state(health, pose, PoseSource::kSentryLidar, 1500000000LL);
    EXPECT_EQ(message.state, message.STATE_DEGRADED);
    EXPECT_EQ(message.source, message.SOURCE_SENTRY_LIDAR);
    EXPECT_TRUE(message.pose_valid);
    EXPECT_DOUBLE_EQ(message.confidence, 0.42);
    EXPECT_EQ(message.header.frame_id, "map");
    EXPECT_EQ(message.header.stamp.sec, 1);
    EXPECT_EQ(message.header.stamp.nanosec, 500000000U);
    EXPECT_DOUBLE_EQ(message.pose.pose.position.x, 1.0);
    EXPECT_EQ(message.reason, "残差偏大");
}

TEST(ConversionTest, PoseValidFollowsHealthState) {
    // 审计 B2：只有完成校正的状态才允许下游消费 pose。
    struct Case {
        HealthState state;
        bool expected_valid;
    };
    const Case cases[] = {
        {HealthState::kInitializing, false},
        {HealthState::kOk, true},
        {HealthState::kDegraded, true},
        {HealthState::kLost, false},
    };

    Pose3D pose;
    pose.parent_frame = "map";
    pose.child_frame = "base_footprint";
    pose.transform = Eigen::Isometry3d::Identity();

    for (const auto& item : cases) {
        HealthStatus health;
        health.state = item.state;
        health.reason = foray::localization::to_string(item.state);

        const auto message = to_msg_state(health, pose, PoseSource::kSentryLidar, 1000000000LL);
        EXPECT_EQ(message.pose_valid, item.expected_valid) << "state=" << message.state;
    }
}

TEST(ConversionTest, PoseRoundTrip) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.translation() = Eigen::Vector3d(-1.5, 0.25, 0.0);

    const auto message = to_msg_pose(transform);
    const Eigen::Isometry3d restored = foray::localization_ros::to_eigen_pose(message);
    EXPECT_LT((restored.translation() - transform.translation()).norm(), 1e-9);
    EXPECT_LT((restored.linear() - transform.linear()).norm(), 1e-9);
}

TEST(ConversionTest, MapMetaAndObstacles) {
    MapManifest manifest;
    manifest.version = "2027_3v3";
    manifest.frame_id = "map";
    manifest.origin = {1.0, -2.0, 0.0};
    manifest.resolution = 0.05;
    manifest.width = 240;
    manifest.height = 160;
    manifest.pcd_sha256 = std::string(64, 'a');
    manifest.grid_sha256 = std::string(64, 'b');
    manifest.source = "sim";

    const auto meta = to_msg_map_meta(manifest, 1000LL);
    EXPECT_EQ(meta.version, "2027_3v3");
    EXPECT_EQ(meta.header.frame_id, "map");
    EXPECT_EQ(meta.width, 240U);
    EXPECT_DOUBLE_EQ(meta.origin.x, 1.0);

    ResidualCluster cluster;
    cluster.centroid = Eigen::Vector3d(1.0, 2.0, 0.3);
    cluster.min_corner = Eigen::Vector3f(0.5F, 1.5F, 0.0F);
    cluster.max_corner = Eigen::Vector3f(1.5F, 2.5F, 0.6F);
    cluster.point_count = 12;
    cluster.confidence = 0.9;
    cluster.mean_residual_m = 0.4;

    const auto obstacles = to_msg_obstacles({cluster}, "map", 2000LL);
    ASSERT_EQ(obstacles.obstacles.size(), 1U);
    EXPECT_EQ(obstacles.header.frame_id, "map");
    EXPECT_EQ(obstacles.obstacles[0].point_count, 12U);
    EXPECT_NEAR(obstacles.obstacles[0].size.x, 1.0, 1e-5);
}

TEST(ConversionTest, TimestampRoundTrip) {
    const auto stamp = to_msg_time(1234567890123LL);
    EXPECT_EQ(foray::localization_ros::to_ns(stamp), 1234567890123LL);
}

} // namespace
