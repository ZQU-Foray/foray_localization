// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/conversion.hpp"

#include <cmath>

namespace foray::localization_ros {

builtin_interfaces::msg::Time to_msg_time(std::int64_t stamp_ns) {
    builtin_interfaces::msg::Time stamp;
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    stamp.sec = static_cast<std::int32_t>(stamp_ns / kNanosecondsPerSecond);
    stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % kNanosecondsPerSecond);
    return stamp;
}

std::int64_t to_ns(const builtin_interfaces::msg::Time& stamp) {
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
           static_cast<std::int64_t>(stamp.nanosec);
}

geometry_msgs::msg::Pose to_msg_pose(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.linear());
    const Eigen::Quaterniond normalized = quaternion.normalized();
    pose.orientation.x = normalized.x();
    pose.orientation.y = normalized.y();
    pose.orientation.z = normalized.z();
    pose.orientation.w = normalized.w();
    return pose;
}

Eigen::Isometry3d to_eigen_pose(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z);
    if (quaternion.norm() > 1e-9) {
        transform.linear() = quaternion.normalized().toRotationMatrix();
    }
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return transform;
}

std::uint8_t to_msg_state(foray::localization::HealthState state) {
    switch (state) {
    case foray::localization::HealthState::kInitializing:
        return LocalizationStateMsg::STATE_INITIALIZING;
    case foray::localization::HealthState::kOk:
        return LocalizationStateMsg::STATE_OK;
    case foray::localization::HealthState::kDegraded:
        return LocalizationStateMsg::STATE_DEGRADED;
    case foray::localization::HealthState::kLost:
        return LocalizationStateMsg::STATE_LOST;
    }
    return LocalizationStateMsg::STATE_LOST;
}

std::uint8_t to_msg_source(foray::localization::PoseSource source) {
    switch (source) {
    case foray::localization::PoseSource::kSentryLidar:
        return LocalizationStateMsg::SOURCE_SENTRY_LIDAR;
    case foray::localization::PoseSource::kExternalOdom:
        return LocalizationStateMsg::SOURCE_EXTERNAL_ODOM;
    case foray::localization::PoseSource::kUnknown:
        return LocalizationStateMsg::SOURCE_UNKNOWN;
    }
    return LocalizationStateMsg::SOURCE_UNKNOWN;
}

LocalizationStateMsg to_msg_state(const foray::localization::HealthStatus& health,
                                  const foray::localization::Pose3D& pose,
                                  foray::localization::PoseSource source, std::int64_t now_ns) {
    LocalizationStateMsg message;
    message.header.stamp = to_msg_time(now_ns);
    message.header.frame_id = pose.parent_frame;
    message.pose.pose = to_msg_pose(pose.transform);
    message.pose.covariance.fill(0.0);
    // 只有 OK/DEGRADED 的 pose 可被下游消费；INITIALIZING 还没完成热启动、LOST 已停更
    // map->odom，两者的 pose 都只是诊断值（审计 B2：禁止静默外推）。
    message.pose_valid = health.state == foray::localization::HealthState::kOk ||
                         health.state == foray::localization::HealthState::kDegraded;
    message.confidence = health.confidence;
    message.age_sec = health.age_sec;
    message.state = to_msg_state(health.state);
    message.source = to_msg_source(source);
    message.reason = health.reason;
    return message;
}

MapMetaMsg to_msg_map_meta(const foray::localization::MapManifest& manifest, std::int64_t now_ns) {
    MapMetaMsg message;
    message.header.stamp = to_msg_time(now_ns);
    message.header.frame_id = manifest.frame_id;
    message.version = manifest.version;
    message.origin.x = manifest.origin[0];
    message.origin.y = manifest.origin[1];
    message.origin.z = manifest.origin[2];
    message.resolution = manifest.resolution;
    message.width = static_cast<std::uint32_t>(manifest.width);
    message.height = static_cast<std::uint32_t>(manifest.height);
    message.pcd_sha256 = manifest.pcd_sha256;
    message.grid_sha256 = manifest.grid_sha256;
    message.source = manifest.source;
    return message;
}

DynamicObstacleArrayMsg
to_msg_obstacles(const std::vector<foray::localization_map::ResidualCluster>& clusters,
                 const std::string& frame_id, std::int64_t stamp_ns) {
    DynamicObstacleArrayMsg message;
    message.header.stamp = to_msg_time(stamp_ns);
    message.header.frame_id = frame_id;
    message.obstacles.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        foray_localization_ros::msg::DynamicObstacle obstacle;
        obstacle.centroid.x = cluster.centroid.x();
        obstacle.centroid.y = cluster.centroid.y();
        obstacle.centroid.z = cluster.centroid.z();
        obstacle.size.x = cluster.max_corner.x() - cluster.min_corner.x();
        obstacle.size.y = cluster.max_corner.y() - cluster.min_corner.y();
        obstacle.size.z = cluster.max_corner.z() - cluster.min_corner.z();
        obstacle.point_count = static_cast<std::uint32_t>(cluster.point_count);
        obstacle.confidence = cluster.confidence;
        obstacle.mean_residual_m = cluster.mean_residual_m;
        message.obstacles.push_back(obstacle);
    }
    return message;
}

} // namespace foray::localization_ros
