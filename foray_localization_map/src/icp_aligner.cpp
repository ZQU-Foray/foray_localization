// Copyright 2026 ZQU-Foray

#include "foray/localization_map/icp_aligner.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace foray::localization_map {
namespace {

struct CellKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellHash {
    std::size_t operator()(const CellKey& key) const {
        const std::uint64_t packed = (static_cast<std::uint64_t>(key.x) << 42U) ^
                                     (static_cast<std::uint64_t>(key.y) << 21U) ^
                                     static_cast<std::uint64_t>(key.z);
        std::uint64_t h = packed * 0x9E3779B97F4A7C15ULL;
        h ^= h >> 29U;
        return static_cast<std::size_t>(h);
    }
};

using CellMap = std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash>;

CellKey cell_of(const Eigen::Vector3f& point, double size) {
    return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size)),
                   static_cast<std::int32_t>(std::floor(point.y() / size)),
                   static_cast<std::int32_t>(std::floor(point.z() / size))};
}

CellMap build_index(const PointCloud& cloud, double size) {
    CellMap index;
    index.reserve(cloud.points.size());
    for (std::uint32_t i = 0; i < cloud.points.size(); ++i) {
        index[cell_of(cloud.points[i], size)].push_back(i);
    }
    return index;
}

bool nearest(const CellMap& index, const PointCloud& target, const Eigen::Vector3f& query,
             double size, double max_distance, std::uint32_t* index_out, double* distance_out) {
    const CellKey center = cell_of(query, size);
    double best = max_distance * max_distance;
    bool found = false;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const auto it = index.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
                if (it == index.end()) {
                    continue;
                }
                for (const std::uint32_t candidate : it->second) {
                    const double distance =
                        (target.points[candidate] - query).cast<double>().squaredNorm();
                    if (distance < best) {
                        best = distance;
                        *index_out = candidate;
                        found = true;
                    }
                }
            }
        }
    }
    if (found) {
        *distance_out = std::sqrt(best);
    }
    return found;
}

} // namespace

IcpAligner::IcpAligner(AlignOptions options) : options_(options) {}

AlignResult IcpAligner::align(const PointCloud& source, const PointCloud& target,
                              const Eigen::Isometry3d& initial_guess) {
    AlignResult result;
    result.transform = initial_guess;

    if (source.empty() || target.empty()) {
        result.error = "配准输入点云为空";
        return result;
    }
    const double cell_size = std::max(1e-3, options_.max_correspondence_distance_m);
    const CellMap index = build_index(target, cell_size);

    Eigen::Isometry3d current = initial_guess;
    std::vector<Eigen::Vector3f> transformed;
    transformed.reserve(source.points.size());

    for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
        transformed.clear();
        for (const Eigen::Vector3f& point : source.points) {
            transformed.push_back((current * point.cast<double>()).cast<float>());
        }

        std::vector<Eigen::Vector3f> from;
        std::vector<Eigen::Vector3f> to;
        from.reserve(source.points.size());
        to.reserve(source.points.size());
        for (const Eigen::Vector3f& point : transformed) {
            std::uint32_t match = 0;
            double distance = 0.0;
            if (nearest(index, target, point, cell_size, options_.max_correspondence_distance_m,
                        &match, &distance)) {
                from.push_back(point);
                to.push_back(target.points[match]);
            }
        }
        const std::size_t minimum = std::max<std::size_t>(10U, source.points.size() / 20U);
        if (from.size() < minimum) {
            result.error = "有效对应点不足（" + std::to_string(from.size()) + " < " +
                           std::to_string(minimum) + "）";
            return result;
        }

        Eigen::Vector3d mean_from = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_to = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < from.size(); ++i) {
            mean_from += from[i].cast<double>();
            mean_to += to[i].cast<double>();
        }
        mean_from /= static_cast<double>(from.size());
        mean_to /= static_cast<double>(to.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < from.size(); ++i) {
            covariance +=
                (from[i].cast<double>() - mean_from) * (to[i].cast<double>() - mean_to).transpose();
        }

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
        if (rotation.determinant() < 0.0) {
            Eigen::Matrix3d v = svd.matrixV();
            v.col(2) *= -1.0;
            rotation = v * svd.matrixU().transpose();
        }

        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
        delta.linear() = rotation;
        delta.translation() = mean_to - rotation * mean_from;
        current = delta * current;
        result.iterations = iteration + 1;

        const double angle = Eigen::AngleAxisd(delta.linear()).angle();
        if (delta.translation().norm() < options_.transformation_epsilon_m &&
            angle < options_.transformation_epsilon_m) {
            result.converged = true;
            break;
        }
    }

    std::size_t inliers = 0;
    double residual_sum = 0.0;
    for (const Eigen::Vector3f& point : source.points) {
        const Eigen::Vector3f moved = (current * point.cast<double>()).cast<float>();
        std::uint32_t match = 0;
        double distance = 0.0;
        if (nearest(index, target, moved, cell_size, options_.max_correspondence_distance_m, &match,
                    &distance)) {
            ++inliers;
            residual_sum += distance;
        }
    }

    result.transform = current;
    result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(source.points.size());
    result.mean_residual_m = inliers > 0U ? residual_sum / static_cast<double>(inliers) : 0.0;
    result.fitness = result.inlier_ratio /
                     (1.0 + result.mean_residual_m / options_.max_correspondence_distance_m);
    return result;
}

} // namespace foray::localization_map
