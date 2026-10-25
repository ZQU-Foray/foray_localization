// Copyright 2026 ZQU-Foray
//
// ROS 消息与 core/map 值类型的转换。保持纯函数，便于单测与复用。

#ifndef FORAY_LOCALIZATION_ROS_CONVERSION_HPP_
#define FORAY_LOCALIZATION_ROS_CONVERSION_HPP_

#include "foray/localization/health.hpp"
#include "foray/localization/manifest.hpp"
#include "foray/localization/pose.hpp"
#include "foray/localization_map/residual_clusterer.hpp"

#include "foray_localization_ros/msg/dynamic_obstacle_array.hpp"
#include "foray_localization_ros/msg/localization_state.hpp"
#include "foray_localization_ros/msg/map_meta.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

namespace foray::localization_ros {

using LocalizationStateMsg = foray_localization_ros::msg::LocalizationState;
using MapMetaMsg = foray_localization_ros::msg::MapMeta;
using DynamicObstacleArrayMsg = foray_localization_ros::msg::DynamicObstacleArray;

/// 时间戳（纳秒）与 ROS 时间互转。
builtin_interfaces::msg::Time to_msg_time(std::int64_t stamp_ns);
std::int64_t to_ns(const builtin_interfaces::msg::Time& stamp);

/// SE(3) 与 geometry_msgs::Pose 互转（四元数归一化）。
geometry_msgs::msg::Pose to_msg_pose(const Eigen::Isometry3d& transform);
Eigen::Isometry3d to_eigen_pose(const geometry_msgs::msg::Pose& pose);

/// 健康状态与位姿来源的枚举映射。
std::uint8_t to_msg_state(foray::localization::HealthState state);
std::uint8_t to_msg_source(foray::localization::PoseSource source);

/// 组装 LocalizationState：header.stamp 取 now_ns，pose 取 map 下位姿。
LocalizationStateMsg to_msg_state(const foray::localization::HealthStatus& health,
                                  const foray::localization::Pose3D& pose,
                                  foray::localization::PoseSource source, std::int64_t now_ns);

MapMetaMsg to_msg_map_meta(const foray::localization::MapManifest& manifest, std::int64_t now_ns);

DynamicObstacleArrayMsg
to_msg_obstacles(const std::vector<foray::localization_map::ResidualCluster>& clusters,
                 const std::string& frame_id, std::int64_t stamp_ns);

} // namespace foray::localization_ros

#endif // FORAY_LOCALIZATION_ROS_CONVERSION_HPP_
