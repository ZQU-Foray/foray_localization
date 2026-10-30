// Copyright 2026 ZQU-Foray

#include "foray/localization_map/pcl_gicp_aligner.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include <cstdint>
#include <string>

#include "foray/localization_map/alignment_metrics.hpp"

namespace foray::localization_map {
namespace {

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

PclCloud::Ptr to_pcl(const PointCloud& cloud) {
    PclCloud::Ptr out(new PclCloud());
    out->reserve(cloud.points.size());
    for (const Eigen::Vector3f& point : cloud.points) {
        out->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    out->width = static_cast<std::uint32_t>(out->size());
    out->height = 1;
    return out;
}

} // namespace

PclGicpAligner::PclGicpAligner(AlignOptions options) : options_(options) {}

AlignResult PclGicpAligner::align(const PointCloud& source, const PointCloud& target,
                                  const Eigen::Isometry3d& initial_guess) {
    AlignResult result;
    result.transform = initial_guess;

    if (source.empty() || target.empty()) {
        result.error = "配准输入点云为空";
        return result;
    }

    const PclCloud::Ptr source_pcl = to_pcl(source);
    const PclCloud::Ptr target_pcl = to_pcl(target);

    // 每帧新建对象、不做 target 预处理缓存：PCL 1.12 的 setInputTarget() 会
    // target_covariances_.reset()，协方差仅在为空时重算，而 setInputSource()
    // 不重置 input_covariances_，且没有 getTargetCovariances() 可取回。
    // 复用同一对象跨帧就等于让 source 协方差与新 source 错配（decision.md D21）。
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(options_.max_iterations);
    gicp.setMaxCorrespondenceDistance(options_.max_correspondence_distance_m);
    gicp.setTransformationEpsilon(options_.transformation_epsilon_m);
    // correspondence_randomness 保留 PCL 默认 20：没有实测数据支撑调参，
    // 先不暴露成 ROS 参数，避免凭空多出一个没人验证过的旋钮（D21）。
    gicp.setInputSource(source_pcl);
    gicp.setInputTarget(target_pcl);

    PclCloud aligned;
    gicp.align(aligned, initial_guess.matrix().cast<float>());

    result.transform = gicp.getFinalTransformation().cast<double>();
    result.converged = gicp.hasConverged();
    // PCL 没有迭代次数的 getter；nr_iterations_ 在 GeneralizedIterativeClosestPoint
    // 的 public using 区（/usr/include/pcl-1.12/pcl/registration/gicp.h:59-68），
    // 直接读它比派生一个访问器更轻，版本耦合点集中在这一行。
    result.iterations = static_cast<int>(gicp.nr_iterations_);

    // 质量一律走公共口径：PCL 的 getFitnessScore() 是均方误差（越小越好），
    // 与本项目 fitness（0..1，越大越好）语义相反，绝不能直接填进 AlignResult。
    const AlignmentQuality quality = evaluate_alignment(source, target, result.transform,
                                                        options_.max_correspondence_distance_m);
    result.inlier_ratio = quality.inlier_ratio;
    result.mean_residual_m = quality.mean_residual_m;
    result.fitness = quality.fitness;

    // 失败原因必须写清楚：节点侧只看到「registration rejected」时，
    // 无从区分是没收敛还是压根没对上（decision.md D21）。
    if (result.inlier_ratio <= 0.0) {
        result.error = "GICP 无有效对应（source=" + std::to_string(source.size()) +
                       " target=" + std::to_string(target.size()) +
                       " iterations=" + std::to_string(result.iterations) + "）";
    } else if (!result.converged) {
        result.error = "GICP 未收敛（iterations=" + std::to_string(result.iterations) + "）";
    }
    return result;
}

} // namespace foray::localization_map
