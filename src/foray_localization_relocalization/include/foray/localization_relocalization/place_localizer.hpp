// Copyright 2026 ZQU-Foray
//
// 可选的地点识别后端（D10）：默认关闭，纯 CPU ONNX Runtime。
// 未编译 ONNX Runtime 时接口仍可链接，但 estimate() 明确返回不可用。

#ifndef FORAY_LOCALIZATION_RELOCALIZATION_PLACE_LOCALIZER_HPP_
#define FORAY_LOCALIZATION_RELOCALIZATION_PLACE_LOCALIZER_HPP_

#include "foray/localization_map/point_cloud.hpp"
#include "foray/localization_map/prior_map.hpp"

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace foray::localization_relocalization {

struct PlaceConfig {
    bool enabled{false};
    std::string model_path;
    double score_threshold{0.5};
    int image_size{64};
};

struct PlaceCandidate {
    Eigen::Isometry3d map_to_base{Eigen::Isometry3d::Identity()};
    double score{0.0};
};

struct PlaceResult {
    bool ok{false};
    std::vector<PlaceCandidate> candidates;
    std::string error;
};

// 返回是否在编译期链接了 ONNX Runtime（纯 CPU）。
bool onnx_backend_available();

class PlaceLocalizer {
  public:
    explicit PlaceLocalizer(PlaceConfig config = {});

    const PlaceConfig& config() const;
    void set_config(const PlaceConfig& config);

    // 失败时 error 说明原因（未启用 / 模型缺失 / ONNX 不可用 / 分数不足）。
    PlaceResult estimate(const foray::localization_map::PointCloud& cloud,
                         const foray::localization_map::PriorMap& map);

  private:
    PlaceConfig config_;
};

} // namespace foray::localization_relocalization

#endif // FORAY_LOCALIZATION_RELOCALIZATION_PLACE_LOCALIZER_HPP_
