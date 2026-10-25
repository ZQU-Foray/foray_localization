// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcd_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace foray::localization_map {
namespace {

std::vector<std::string> split_tokens(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

struct PcdHeader {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<std::string> types;
    std::vector<int> counts;
    std::size_t width{0};
    std::size_t height{0};
    std::size_t points{0};
    std::string data;
};

bool parse_header(std::ifstream& file, PcdHeader* header, std::string* error) {
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const std::vector<std::string> tokens = split_tokens(line);
        if (tokens.empty()) {
            continue;
        }
        const std::string& key = tokens[0];
        if (key == "FIELDS") {
            header->fields.assign(tokens.begin() + 1, tokens.end());
        } else if (key == "SIZE") {
            header->sizes.clear();
            for (std::size_t i = 1; i < tokens.size(); ++i) {
                header->sizes.push_back(std::stoi(tokens[i]));
            }
        } else if (key == "TYPE") {
            header->types.assign(tokens.begin() + 1, tokens.end());
        } else if (key == "COUNT") {
            header->counts.clear();
            for (std::size_t i = 1; i < tokens.size(); ++i) {
                header->counts.push_back(std::stoi(tokens[i]));
            }
        } else if (key == "WIDTH") {
            header->width = tokens.size() >= 2 ? std::stoull(tokens[1]) : 0U;
        } else if (key == "HEIGHT") {
            header->height = tokens.size() >= 2 ? std::stoull(tokens[1]) : 0U;
        } else if (key == "POINTS") {
            header->points = tokens.size() >= 2 ? std::stoull(tokens[1]) : 0U;
        } else if (key == "DATA") {
            header->data = tokens.size() >= 2 ? tokens[1] : std::string();
            if (header->points == 0U) {
                header->points = header->width * header->height;
            }
            return true;
        }
    }
    *error = "PCD 头缺少 DATA 行";
    return false;
}

struct FieldLayout {
    int x_offset{-1};
    int y_offset{-1};
    int z_offset{-1};
    int x_size{0};
    int y_size{0};
    int z_size{0};
    std::size_t x_type_index{0};
    std::size_t y_type_index{0};
    std::size_t z_type_index{0};
    std::size_t stride{0};
};

bool field_layout(const PcdHeader& header, FieldLayout* layout, std::string* error) {
    const std::size_t count = header.fields.size();
    if (header.sizes.size() != count || header.types.size() != count ||
        header.counts.size() != count) {
        *error = "PCD 头 FIELDS/SIZE/TYPE/COUNT 数量不一致";
        return false;
    }

    int offset = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const int bytes = header.sizes[i] * header.counts[i];
        if (header.fields[i] == "x") {
            layout->x_offset = offset;
            layout->x_size = header.sizes[i];
            layout->x_type_index = i;
        } else if (header.fields[i] == "y") {
            layout->y_offset = offset;
            layout->y_size = header.sizes[i];
            layout->y_type_index = i;
        } else if (header.fields[i] == "z") {
            layout->z_offset = offset;
            layout->z_size = header.sizes[i];
            layout->z_type_index = i;
        }
        offset += bytes;
    }
    layout->stride = static_cast<std::size_t>(offset);
    if (layout->x_offset < 0 || layout->y_offset < 0 || layout->z_offset < 0) {
        *error = "PCD 缺少 x/y/z 字段";
        return false;
    }
    return true;
}

bool read_numeric(const char* base, int offset, int size, const std::string& type, float* value) {
    if (type == "F" && size == 4) {
        float v = 0.0F;
        std::memcpy(&v, base + offset, sizeof(float));
        *value = v;
        return true;
    }
    if (type == "F" && size == 8) {
        double v = 0.0;
        std::memcpy(&v, base + offset, sizeof(double));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "I" && size == 2) {
        std::int16_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "I" && size == 4) {
        std::int32_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    if (type == "U" && size == 2) {
        std::uint16_t v = 0;
        std::memcpy(&v, base + offset, sizeof(v));
        *value = static_cast<float>(v);
        return true;
    }
    return false;
}

} // namespace

PcdLoadResult load_pcd(const std::filesystem::path& path) {
    PcdLoadResult result;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        result.error = "无法打开 PCD 文件: " + path.string();
        return result;
    }

    PcdHeader header;
    if (!parse_header(file, &header, &result.error)) {
        return result;
    }
    if (header.data != "ascii" && header.data != "binary") {
        result.error =
            "不支持的 PCD DATA 格式: " + header.data + "（仅支持 ascii/binary，请导出未压缩格式）";
        return result;
    }

    FieldLayout layout;
    if (!field_layout(header, &layout, &result.error)) {
        return result;
    }
    if (layout.stride == 0U) {
        result.error = "PCD 点步长为 0";
        return result;
    }

    const std::size_t point_count = header.points;
    if (point_count == 0U) {
        result.error = "PCD 点数声明为 0";
        return result;
    }

    result.cloud.points.reserve(point_count);

    if (header.data == "ascii") {
        std::string line;
        std::size_t index = 0;
        while (index < point_count && std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            const std::vector<std::string> tokens = split_tokens(line);
            if (tokens.size() < header.fields.size()) {
                continue;
            }
            const float x = std::stof(tokens[layout.x_type_index]);
            const float y = std::stof(tokens[layout.y_type_index]);
            const float z = std::stof(tokens[layout.z_type_index]);
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                result.cloud.points.emplace_back(x, y, z);
            }
            ++index;
        }
    } else {
        std::vector<char> buffer(layout.stride * point_count);
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        if (static_cast<std::size_t>(file.gcount()) != buffer.size()) {
            result.error = "PCD 二进制数据长度不足";
            return result;
        }
        for (std::size_t i = 0; i < point_count; ++i) {
            const char* base = buffer.data() + i * layout.stride;
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
            if (!read_numeric(base, layout.x_offset, layout.x_size,
                              header.types[layout.x_type_index], &x) ||
                !read_numeric(base, layout.y_offset, layout.y_size,
                              header.types[layout.y_type_index], &y) ||
                !read_numeric(base, layout.z_offset, layout.z_size,
                              header.types[layout.z_type_index], &z)) {
                result.error =
                    "PCD 的 x/y/z 类型不受支持（仅支持 F 的 4/8 字节与 I/U 的 2/4 字节）";
                return result;
            }
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                result.cloud.points.emplace_back(x, y, z);
            }
        }
    }

    if (result.cloud.points.empty()) {
        result.error = "PCD 解析后没有有效点";
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace foray::localization_map
