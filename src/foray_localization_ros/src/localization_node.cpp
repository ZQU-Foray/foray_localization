// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/localization_node.hpp"

#include "foray/localization_ros/adapters.hpp"
#include "foray/localization_ros/conversion.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace foray::localization_ros {
namespace {

constexpr double kDefaultStateRateHz = 50.0;
constexpr double kDefaultTfRateHz = 20.0;
constexpr double kDefaultAlignRateHz = 5.0;
constexpr double kDefaultObstacleRateHz = 10.0;

foray::localization::PoseSourcePolicy parse_policy(const std::string& value) {
    if (value == "lidar") {
        return foray::localization::PoseSourcePolicy::kLidar;
    }
    if (value == "external") {
        return foray::localization::PoseSourcePolicy::kExternal;
    }
    return foray::localization::PoseSourcePolicy::kAuto;
}

} // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("foray_localization", options), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_), aligner_(foray::localization_map::AlignOptions{}),
      clusterer_(foray::localization_map::ResidualClustererConfig{}), health_() {
    map_manifest_ = declare_parameter<std::string>("map_manifest", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.1);
    map_voxel_leaf_m_ = declare_parameter<double>("map_voxel_leaf_m", 0.25);
    residual_threshold_m_ = declare_parameter<double>("residual_threshold_m", 0.3);
    cluster_tolerance_m_ = declare_parameter<double>("cluster_tolerance_m", 0.3);
    verify_map_hashes_ = declare_parameter<bool>("verify_map_hashes", true);
    pose_source_param_ = declare_parameter<std::string>("pose_source", "auto");

    health_config_.ok_mean_residual_m = declare_parameter<double>("ok_mean_residual_m", 0.10);
    health_config_.degraded_mean_residual_m =
        declare_parameter<double>("degraded_mean_residual_m", 0.20);
    health_config_.ok_inlier_ratio = declare_parameter<double>("ok_inlier_ratio", 0.55);
    health_config_.degraded_inlier_ratio = declare_parameter<double>("degraded_inlier_ratio", 0.35);
    health_config_.lost_timeout_sec = declare_parameter<double>("lost_timeout_sec", 1.0);
    health_config_.odom_timeout_sec = declare_parameter<double>("odom_timeout_sec", 0.5);
    health_config_.warmup_frames = declare_parameter<int>("warmup_frames", 3);
    health_config_.recover_frames = declare_parameter<int>("recover_frames", 3);
    health_ = foray::localization::HealthMonitor(health_config_);

    foray::localization_map::AlignOptions align_options;
    align_options.max_iterations = declare_parameter<int>("align_max_iterations", 10);
    align_options.max_correspondence_distance_m =
        declare_parameter<double>("align_max_correspondence_m", 0.5);
    aligner_ = foray::localization_map::IcpAligner(align_options);

    foray::localization_map::ResidualClustererConfig cluster_config;
    cluster_config.residual_threshold_m = residual_threshold_m_;
    cluster_config.cluster_tolerance_m = cluster_tolerance_m_;
    cluster_config.min_cluster_size = declare_parameter<int>("min_cluster_size", 5);
    cluster_config.max_cluster_size = declare_parameter<int>("max_cluster_size", 500);
    clusterer_ = foray::localization_map::ResidualClusterer(cluster_config);

    policy_ = parse_policy(pose_source_param_);
    active_source_ = policy_ == foray::localization::PoseSourcePolicy::kExternal
                         ? foray::localization::PoseSource::kExternalOdom
                         : foray::localization::PoseSource::kSentryLidar;

    const auto state_rate = declare_parameter<double>("state_rate_hz", kDefaultStateRateHz);
    const auto tf_rate = declare_parameter<double>("tf_rate_hz", kDefaultTfRateHz);
    const auto align_rate = declare_parameter<double>("align_rate_hz", kDefaultAlignRateHz);
    const auto obstacle_rate =
        declare_parameter<double>("obstacle_rate_hz", kDefaultObstacleRateHz);

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto external_topic =
        declare_parameter<std::string>("external_pose_topic", "/localization/external_pose");

    const auto state_qos = rclcpp::QoS(10).reliable().transient_local();
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto obstacle_qos = rclcpp::QoS(10).reliable();

    state_pub_ = create_publisher<foray_localization_ros::msg::LocalizationState>(
        "/localization/state", state_qos);
    map_meta_pub_ =
        create_publisher<foray_localization_ros::msg::MapMeta>("/localization/map_meta", map_qos);
    obstacle_pub_ = create_publisher<foray_localization_ros::msg::DynamicObstacleArray>(
        "/localization/dynamic_obstacles", obstacle_qos);

    if (policy_ != foray::localization::PoseSourcePolicy::kExternal) {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationNode::on_cloud, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::QoS(50),
            std::bind(&LocalizationNode::on_odometry, this, std::placeholders::_1));
    }
    if (policy_ != foray::localization::PoseSourcePolicy::kLidar) {
        external_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            external_topic, rclcpp::QoS(20),
            std::bind(&LocalizationNode::on_external_pose, this, std::placeholders::_1));
    }

    relocalize_srv_ = create_service<foray_localization_ros::srv::Relocalize>(
        "/localization/relocalize", std::bind(&LocalizationNode::on_relocalize, this,
                                              std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    load_map();

    state_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
                                     std::bind(&LocalizationNode::on_state_timer, this));
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate)),
                                  std::bind(&LocalizationNode::on_tf_timer, this));
    align_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, align_rate)),
                                     std::bind(&LocalizationNode::on_align_timer, this));
    obstacle_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, obstacle_rate)),
                          std::bind(&LocalizationNode::on_obstacle_timer, this));
}

void LocalizationNode::load_map() {
    if (map_manifest_.empty()) {
        RCLCPP_ERROR(get_logger(), "map_manifest 未设置，定位节点无法启动地图配准");
        return;
    }
    foray::localization_map::PriorMap map;
    const auto result =
        foray::localization_map::load_prior_map(map_manifest_, &map, verify_map_hashes_);
    if (!result.ok) {
        RCLCPP_ERROR(get_logger(), "加载先验地图失败: %s", result.error.c_str());
        return;
    }
    if (map_voxel_leaf_m_ > 0.0) {
        map.cloud = foray::localization_map::voxel_downsample(map.cloud, map_voxel_leaf_m_);
    }
    map_ = std::move(map);
    has_map_ = true;
    RCLCPP_INFO(get_logger(), "先验地图已加载: version=%s pcd=%zu grid=%dx%d",
                map_.manifest.version.c_str(), map_.cloud.size(), map_.grid.width,
                map_.grid.height);

    map_meta_pub_->publish(to_msg_map_meta(map_.manifest, now().nanoseconds()));
}

void LocalizationNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    std::string error;
    foray::localization_map::PointCloud points = to_point_cloud(*cloud, &error);
    if (points.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃无效点云: %s", error.c_str());
        return;
    }
    if (voxel_leaf_m_ > 0.0) {
        points = foray::localization_map::voxel_downsample(points, voxel_leaf_m_);
    }

    Eigen::Isometry3d base_from_cloud = Eigen::Isometry3d::Identity();
    if (!cloud->header.frame_id.empty() && cloud->header.frame_id != base_frame_) {
        try {
            const auto transform = tf_buffer_.lookupTransform(base_frame_, cloud->header.frame_id,
                                                              rclcpp::Time(cloud->header.stamp),
                                                              std::chrono::milliseconds(100));
            const auto& stamped = transform.transform;
            const Eigen::Quaterniond quaternion(stamped.rotation.w, stamped.rotation.x,
                                                stamped.rotation.y, stamped.rotation.z);
            base_from_cloud.setIdentity();
            if (quaternion.norm() > 1e-9) {
                base_from_cloud.linear() = quaternion.normalized().toRotationMatrix();
            }
            base_from_cloud.translation() = Eigen::Vector3d(
                stamped.translation.x, stamped.translation.y, stamped.translation.z);
        } catch (const tf2::TransformException& exception) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云 TF %s->%s 失败: %s",
                                 cloud->header.frame_id.c_str(), base_frame_.c_str(),
                                 exception.what());
            return;
        }
    }
    points = foray::localization_map::transform_points(points, base_from_cloud);

    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(cloud->header.stamp, &stamp_ns)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = std::move(points);
    latest_cloud_stamp_ns_ = stamp_ns;
    has_cloud_ = true;
}

void LocalizationNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
    std::string error;
    if (!validate_odometry(*odometry, &error)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃无效里程计: %s", error.c_str());
        return;
    }
    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(odometry->header.stamp, &stamp_ns)) {
        return;
    }
    Eigen::Isometry3d odom_to_base;
    if (!to_isometry(odometry->pose.pose, &odom_to_base)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    odom_to_base_ = odom_to_base;
    last_odom_ns_ = stamp_ns;
    has_odom_ = true;
}

void LocalizationNode::on_external_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) {
    if (policy_ == foray::localization::PoseSourcePolicy::kLidar) {
        return;
    }
    std::string error;
    if (!validate_header(pose->header, map_frame_.c_str(), &error)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃外部位姿: %s", error.c_str());
        return;
    }
    std::int64_t stamp_ns = 0;
    if (!to_timestamp_ns(pose->header.stamp, &stamp_ns)) {
        return;
    }
    Eigen::Isometry3d map_to_base;
    if (!to_isometry(pose->pose.pose, &map_to_base)) {
        return;
    }

    foray::localization::RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = true;
    sample.mean_residual_m = 0.0;
    sample.inlier_ratio = 1.0;
    sample.fitness = 1.0;

    std::lock_guard<std::mutex> lock(mutex_);
    map_to_base_ = map_to_base;
    map_to_odom_ = map_to_base;
    odom_to_base_ = Eigen::Isometry3d::Identity();
    last_odom_ns_ = stamp_ns;
    has_odom_ = true;
    last_valid_ns_ = stamp_ns;
    active_source_ = foray::localization::PoseSource::kExternalOdom;
    health_status_ = health_.update(sample, stamp_ns, stamp_ns);
}

void LocalizationNode::on_align_timer() {
    if (policy_ == foray::localization::PoseSourcePolicy::kExternal) {
        return;
    }

    foray::localization_map::PointCloud cloud;
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    std::int64_t stamp_ns = 0;
    std::int64_t odom_ns = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_map_ || !has_cloud_ || !has_odom_) {
            return;
        }
        cloud = latest_cloud_;
        stamp_ns = latest_cloud_stamp_ns_;
        odom_ns = last_odom_ns_;
        if (relocalize_pending_.exchange(false)) {
            guess = pending_map_to_base_;
        } else {
            guess = map_to_odom_ * odom_to_base_;
        }
    }

    const auto result = aligner_.align(cloud, map_.cloud, guess);

    std::lock_guard<std::mutex> lock(mutex_);
    foray::localization::RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = result.inlier_ratio > 0.0;
    sample.mean_residual_m = result.mean_residual_m;
    sample.inlier_ratio = result.inlier_ratio;
    sample.fitness = result.fitness;
    health_status_ = health_.update(sample, stamp_ns, odom_ns);

    // LOST 粘滞：只有人工重定位 reset 后才重新接受配准结果。
    if (health_status_.state != foray::localization::HealthState::kLost &&
        result.inlier_ratio > 0.0) {
        map_to_base_ = result.transform;
        map_to_odom_ = map_to_base_ * odom_to_base_.inverse();
        last_valid_ns_ = stamp_ns;
    }
    has_cloud_ = false;
}

void LocalizationNode::publish_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::int64_t now_ns = now().nanoseconds();
    // 外部模式没有配准帧驱动，按节拍喂健康状态；里程计断流时任何模式都必须转 LOST。
    if (policy_ == foray::localization::PoseSourcePolicy::kExternal) {
        foray::localization::RegistrationSample sample;
        sample.stamp_ns = now_ns;
        sample.valid = last_odom_ns_ > 0;
        sample.mean_residual_m = 0.0;
        sample.inlier_ratio = sample.valid ? 1.0 : 0.0;
        health_status_ = health_.update(sample, now_ns, last_odom_ns_);
    } else if (has_odom_ && health_config_.odom_timeout_sec > 0.0 &&
               static_cast<double>(now_ns - last_odom_ns_) / 1e9 >
                   health_config_.odom_timeout_sec) {
        foray::localization::RegistrationSample sample;
        sample.stamp_ns = now_ns;
        sample.valid = false;
        sample.mean_residual_m = 0.0;
        sample.inlier_ratio = 0.0;
        health_status_ = health_.update(sample, now_ns, last_odom_ns_);
    }

    foray::localization::Pose3D pose;
    pose.parent_frame = map_frame_;
    pose.child_frame = base_frame_;
    pose.transform = policy_ == foray::localization::PoseSourcePolicy::kExternal
                         ? map_to_base_
                         : map_to_odom_ * odom_to_base_;

    foray::localization::HealthStatus status = health_status_;
    status.age_sec = last_valid_ns_ > 0 ? static_cast<double>(now_ns - last_valid_ns_) / 1e9 : 0.0;
    if (status.state == foray::localization::HealthState::kLost) {
        status.confidence = 0.0;
    }
    state_pub_->publish(to_msg_state(status, pose, active_source_, now_ns));
}

void LocalizationNode::publish_tf() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (health_status_.state == foray::localization::HealthState::kLost || !has_odom_) {
        return;
    }
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    const Eigen::Isometry3d& transform = map_to_odom_;
    message.transform.translation.x = transform.translation().x();
    message.transform.translation.y = transform.translation().y();
    message.transform.translation.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.linear());
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    tf_broadcaster_->sendTransform(message);
}

void LocalizationNode::on_state_timer() {
    publish_state();
}

void LocalizationNode::on_tf_timer() {
    publish_tf();
}

void LocalizationNode::on_obstacle_timer() {
    foray::localization_map::PointCloud cloud;
    Eigen::Isometry3d current_map_to_base;
    std::int64_t stamp_ns = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_map_ || !has_cloud_) {
            return;
        }
        cloud = latest_cloud_;
        current_map_to_base = map_to_base_;
        stamp_ns = latest_cloud_stamp_ns_;
    }
    // has_cloud_ 会被配准线程消费，这里用时间戳判断数据是否仍然新鲜。
    if (now().nanoseconds() - stamp_ns > 1000000000LL) {
        return;
    }
    const auto in_map = foray::localization_map::transform_points(cloud, current_map_to_base);
    const auto clusters = clusterer_.cluster(in_map, map_.cloud);
    obstacle_pub_->publish(to_msg_obstacles(clusters, map_frame_, stamp_ns));
}

void LocalizationNode::on_relocalize(
    const std::shared_ptr<foray_localization_ros::srv::Relocalize::Request> request,
    std::shared_ptr<foray_localization_ros::srv::Relocalize::Response> response) {
    if (request->mode == foray_localization_ros::srv::Relocalize::Request::MODE_PLACE) {
        response->accepted = false;
        response->message = "place 后端默认关闭（D10）；请使用 MODE_GEOMETRIC 给定初值";
        return;
    }

    Eigen::Isometry3d initial;
    if (!to_isometry(request->initial_pose.pose, &initial)) {
        response->accepted = false;
        response->message = "初始位姿含非法数值或零四元数";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_map_to_base_ = initial;
        relocalize_pending_ = true;
        health_.reset();
        health_status_ = foray::localization::HealthStatus{};
        health_status_.state = foray::localization::HealthState::kInitializing;
        map_to_base_ = initial;
        map_to_odom_ = initial * odom_to_base_.inverse();
    }
    response->accepted = true;
    response->message = "已接受初始位姿，下一帧按该初值配准";
    RCLCPP_INFO(get_logger(), "收到手动重定位请求");
}

} // namespace foray::localization_ros
