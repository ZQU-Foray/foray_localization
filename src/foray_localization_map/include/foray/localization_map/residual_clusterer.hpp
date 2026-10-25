// Copyright 2026 ZQU-Foray
//
// 配准残差聚类：把“落在地图之外的观测点”聚成无标号簇，作为动态物体
// 候选（decision.md D6）。标签由下游 vision/decision 给出，本层只给几何与置信度。

#ifndef FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_
#define FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_

#include "foray/localization_map/point_cloud.hpp"

#include <Eigen/Core>

#include <vector>

namespace foray::localization_map {

struct ResidualClustererConfig {
    /// 点到地图最近点的欧氏距离超过该值即为残差点（米）。
    double residual_threshold_m{0.3};
    /// 同一簇内相邻点的最大间距（米）。
    double cluster_tolerance_m{0.3};
    /// 小于该点数的簇视为噪声丢弃。
    int min_cluster_size{5};
    /// 大于该点数的簇不输出，避免把整片地图过期误报成动态物体。
    int max_cluster_size{500};
};

struct ResidualCluster {
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    int point_count{0};
    Eigen::Vector3f min_corner{Eigen::Vector3f::Zero()};
    Eigen::Vector3f max_corner{Eigen::Vector3f::Zero()};
    double mean_residual_m{0.0};
    /// 0..1，仅由簇的点数相对 min_cluster_size 决定，不含语义置信度。
    double confidence{0.0};
};

class ResidualClusterer {
  public:
    explicit ResidualClusterer(ResidualClustererConfig config = ResidualClustererConfig{});

    /// 在 target 地图上为 source 的每个点找最近邻，残差点做欧氏聚类。
    std::vector<ResidualCluster> cluster(const PointCloud& source, const PointCloud& target) const;

  private:
    ResidualClustererConfig config_;
};

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_RESIDUAL_CLUSTERER_HPP_
