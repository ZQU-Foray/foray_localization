// Copyright 2026 ZQU-Foray
//
// 配准后端接口：定位层只依赖这个抽象，PCL GICP / 内置 ICP / 未来的
// small_gicp 后端都可替换，而健康状态机与 ROS 节点不受影响。

#ifndef FORAY_LOCALIZATION_MAP_ALIGNER_HPP_
#define FORAY_LOCALIZATION_MAP_ALIGNER_HPP_

#include "foray/localization_map/point_cloud.hpp"

#include <Eigen/Geometry>

#include <string>

namespace foray::localization_map {

struct AlignOptions {
    /// 最大迭代次数；5Hz 配准下 10 次是精度与实时性的折中（decision.md D2）。
    int max_iterations{10};
    /// 对应点最大距离（米）。超过即视为外点，不参与位姿估计。
    double max_correspondence_distance_m{0.5};
    /// 相邻两轮位姿变化小于该值即认为收敛。
    double transformation_epsilon_m{1e-4};
};

struct AlignResult {
    bool converged{false};
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    /// 0..1：内点比例越高、残差越小，fitness 越接近 1。
    double fitness{0.0};
    double mean_residual_m{0.0};
    double inlier_ratio{0.0};
    int iterations{0};
    std::string error;
};

/// 把 source 点云配准到 target 点云，返回 source->target 的变换。
class Aligner {
  public:
    virtual ~Aligner() = default;

    virtual AlignResult align(const PointCloud& source, const PointCloud& target,
                              const Eigen::Isometry3d& initial_guess) = 0;
};

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_ALIGNER_HPP_
