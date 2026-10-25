// Copyright 2026 ZQU-Foray
//
// 自包含 SHA-256 实现，避免为地图校验引入 OpenSSL 依赖。

#ifndef FORAY_LOCALIZATION_SHA256_HPP_
#define FORAY_LOCALIZATION_SHA256_HPP_

#include <cstddef>
#include <filesystem>
#include <string>

namespace foray::localization {

/// 计算内存块的 sha256，返回 64 位小写十六进制。
std::string sha256_hex(const void* data, std::size_t size);

/// 计算文件 sha256；读取失败返回空字符串并写入 `error`。
std::string sha256_file(const std::filesystem::path& path, std::string* error);

} // namespace foray::localization

#endif // FORAY_LOCALIZATION_SHA256_HPP_
