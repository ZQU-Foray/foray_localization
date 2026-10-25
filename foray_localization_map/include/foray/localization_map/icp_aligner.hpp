// Copyright 2026 ZQU-Foray
//
// 内置点到点 ICP。默认后端，不依赖 PCL/small_gicp，保证在最小构建环境
// 与 CI 中也能完成配准与健康状态验证。精度低于 GICP，赛前应优先启用
// GICP 后端（decision.md D2/D9）。

#ifndef FORAY_LOCALIZATION_MAP_ICP_ALIGNER_HPP_
#define FORAY_LOCALIZATION_MAP_ICP_ALIGNER_HPP_

#include "foray/localization_map/aligner.hpp"

namespace foray::localization_map {

class IcpAligner final : public Aligner {
  public:
    explicit IcpAligner(AlignOptions options = AlignOptions{});

    AlignResult align(const PointCloud& source, const PointCloud& target,
                      const Eigen::Isometry3d& initial_guess) override;

    const AlignOptions& options() const { return options_; }

  private:
    AlignOptions options_;
};

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_ICP_ALIGNER_HPP_
