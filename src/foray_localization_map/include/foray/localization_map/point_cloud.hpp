// Copyright 2026 ZQU-Foray
//
// 与 ROS 无关的点云容器与基础操作。地图与配准后端共用同一表示，
// 避免 PCL/ROS 类型泄漏到核心算法。

#ifndef FORAY_LOCALIZATION_MAP_POINT_CLOUD_HPP_
#define FORAY_LOCALIZATION_MAP_POINT_CLOUD_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace foray::localization_map {

/// 以 float 存储的 XYZ 点云；不保留强度/环号等传感器字段（定位只用几何）。
struct PointCloud {
    std::vector<Eigen::Vector3f> points;

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
};

/// 体素降采样：同一体素内取质心。`leaf_size_m <= 0` 时原样返回。
PointCloud voxel_downsample(const PointCloud& cloud, double leaf_size_m);

/// 按刚体变换逐点变换。
PointCloud transform_points(const PointCloud& cloud, const Eigen::Isometry3d& transform);

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_POINT_CLOUD_HPP_
