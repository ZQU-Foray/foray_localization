// Copyright 2026 ZQU-Foray
//
// 最小 PCD 读取器（ascii / binary，uncompressed）。不依赖 PCL，
// 以便在没有 PCL 的构建环境与 CI 中仍能加载先验地图。

#ifndef FORAY_LOCALIZATION_MAP_PCD_LOADER_HPP_
#define FORAY_LOCALIZATION_MAP_PCD_LOADER_HPP_

#include "foray/localization_map/point_cloud.hpp"

#include <filesystem>
#include <string>

namespace foray::localization_map {

struct PcdLoadResult {
    bool ok{false};
    PointCloud cloud;
    std::string error;
};

/// 读取 PCD 文件。支持 FIELDS 含 x/y/z 的 ascii 与 binary 数据；
/// `binary_compressed` 明确报错（地图构建工具应导出未压缩格式）。
PcdLoadResult load_pcd(const std::filesystem::path& path);

} // namespace foray::localization_map

#endif // FORAY_LOCALIZATION_MAP_PCD_LOADER_HPP_
