// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcd_loader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct TempDir {
    fs::path path;

    TempDir() {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("foray_pcd_test_" + std::to_string(suffix));
        fs::create_directories(path);
    }

    ~TempDir() {
        std::error_code code;
        fs::remove_all(path, code);
    }
};

const char* kAsciiHeader = "# .PCD v0.7\n"
                           "VERSION 0.7\n"
                           "FIELDS x y z\n"
                           "SIZE 4 4 4\n"
                           "TYPE F F F\n"
                           "COUNT 1 1 1\n"
                           "WIDTH 3\n"
                           "HEIGHT 1\n"
                           "POINTS 3\n"
                           "DATA ascii\n";

TEST(PcdLoaderTest, LoadsAsciiCloud) {
    TempDir dir;
    const fs::path file = dir.path / "map.pcd";
    {
        std::ofstream out(file);
        out << kAsciiHeader;
        out << "1.0 2.0 3.0\n";
        out << "-1.5 0.0 0.5\n";
        out << "0.25 0.5 0.75\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.cloud.size(), 3U);
    EXPECT_FLOAT_EQ(result.cloud.points[0].x(), 1.0F);
    EXPECT_FLOAT_EQ(result.cloud.points[1].x(), -1.5F);
    EXPECT_FLOAT_EQ(result.cloud.points[2].z(), 0.75F);
}

TEST(PcdLoaderTest, LoadsBinaryCloud) {
    TempDir dir;
    const fs::path file = dir.path / "map.pcd";
    const std::string header = "VERSION 0.7\n"
                               "FIELDS x y z\n"
                               "SIZE 4 4 4\n"
                               "TYPE F F F\n"
                               "COUNT 1 1 1\n"
                               "WIDTH 2\n"
                               "HEIGHT 1\n"
                               "POINTS 2\n"
                               "DATA binary\n";
    const std::vector<float> points{0.5F, 1.5F, 2.5F, -0.5F, -1.5F, -2.5F};
    {
        std::ofstream out(file, std::ios::binary);
        out.write(header.data(), static_cast<std::streamsize>(header.size()));
        out.write(reinterpret_cast<const char*>(points.data()),
                  static_cast<std::streamsize>(points.size() * sizeof(float)));
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.cloud.size(), 2U);
    EXPECT_FLOAT_EQ(result.cloud.points[1].x(), -0.5F);
    EXPECT_FLOAT_EQ(result.cloud.points[1].z(), -2.5F);
}

TEST(PcdLoaderTest, RejectsCompressedAndMissingFile) {
    TempDir dir;
    const fs::path compressed = dir.path / "compressed.pcd";
    {
        std::ofstream out(compressed);
        out << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
               "POINTS 1\nDATA binary_compressed\n";
    }
    const auto compressed_result = foray::localization_map::load_pcd(compressed);
    EXPECT_FALSE(compressed_result.ok);
    EXPECT_NE(compressed_result.error.find("binary_compressed"), std::string::npos);

    const auto missing_result = foray::localization_map::load_pcd(dir.path / "missing.pcd");
    EXPECT_FALSE(missing_result.ok);
}

TEST(PcdLoaderTest, SkipsInvalidPoints) {
    TempDir dir;
    const fs::path file = dir.path / "nan.pcd";
    {
        std::ofstream out(file);
        out << kAsciiHeader;
        out << "nan 0 0\n";
        out << "1 1 1\n";
        out << "2 2 2\n";
    }

    const auto result = foray::localization_map::load_pcd(file);
    ASSERT_TRUE(result.ok) << result.error;
    // nan 行被跳过，剩下两行有效点。
    EXPECT_EQ(result.cloud.size(), 2U);
}

} // namespace
