// Copyright 2026 ZQU-Foray
//
// PCL GICP 配准后端（默认后端，见 decision.md D21）。
//
// 本头不包含任何 PCL 头：PCL 只在 .cpp 里出现，ROS 包与测试 include 它时
// 不会连带继承 PCL 的编译选项、宏与版本耦合。头文件本身不区分
// FORAY_WITH_PCL：未启用 PCL 的构建也能 include（只是不会构造本类）。

#ifndef FORAY_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_
#define FORAY_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_

#include "foray/localization_map/aligner.hpp"

namespace foray::localization_map {

/// 以 PCL GeneralizedIterativeClosestPoint 实现的配准后端。
class PclGicpAligner final : public Aligner {
  public:
    explicit PclGicpAligner(AlignOptions options = AlignOptions{});

    AlignResult align(const PointCloud& source, const PointCloud& target,
                      const Eigen::Isometry3d& initial_guess) override;

    const AlignOptions& options() const { return options_; }

  private:
    AlignOptions options_;
};

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_PCL_GICP_ALIGNER_HPP_
