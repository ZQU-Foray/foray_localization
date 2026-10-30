// Copyright 2026 ZQU-Foray

#include "foray/localization_map/icp_aligner.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "foray/localization_map/alignment_metrics.hpp"

namespace foray::localization_map {

IcpAligner::IcpAligner(AlignOptions options) : options_(options) {}

AlignResult IcpAligner::align(const PointCloud& source, const PointCloud& target,
                              const Eigen::Isometry3d& initial_guess) {
    AlignResult result;
    result.transform = initial_guess;

    if (source.empty() || target.empty()) {
        result.error = "配准输入点云为空";
        return result;
    }
    const PointCloudIndex index(target, options_.max_correspondence_distance_m);

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
            if (index.nearest(point, options_.max_correspondence_distance_m, &match, &distance)) {
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

    // 质量评估走公共口径（alignment_metrics），与 PCL GICP 后端共用同一套公式；
    // 这里复用迭代内的索引，避免为评估再建一次体素哈希。
    const AlignmentQuality quality =
        evaluate_alignment(source, index, current, options_.max_correspondence_distance_m);
    result.transform = current;
    result.inlier_ratio = quality.inlier_ratio;
    result.mean_residual_m = quality.mean_residual_m;
    result.fitness = quality.fitness;
    return result;
}

} // namespace foray::localization_map
