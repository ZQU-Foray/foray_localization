// Copyright 2026 ZQU-Foray

#include "foray/localization/health.hpp"

#include <gtest/gtest.h>

namespace foray::localization {
namespace {

constexpr Timestamp kSecond = 1'000'000'000;

RegistrationSample make_sample(Timestamp stamp_ns, bool valid = true, double mean_residual_m = 0.05,
                               double inlier_ratio = 0.9) {
    RegistrationSample sample;
    sample.stamp_ns = stamp_ns;
    sample.valid = valid;
    sample.mean_residual_m = mean_residual_m;
    sample.inlier_ratio = inlier_ratio;
    return sample;
}

HealthConfig test_config() {
    HealthConfig config;
    config.warmup_frames = 2;
    config.recover_frames = 2;
    config.lost_timeout_sec = 0.5;
    config.odom_timeout_sec = 0.5;
    config.degraded_hold_sec = 0.0;
    return config;
}

TEST(HealthMonitor, WarmsUpThenStaysOk) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;

    EXPECT_EQ(monitor.update(make_sample(start), start, start).state, HealthState::kInitializing);
    EXPECT_EQ(
        monitor
            .update(make_sample(start + kSecond / 10), start + kSecond / 10, start + kSecond / 10)
            .state,
        HealthState::kOk);

    const HealthStatus status = monitor.update(make_sample(start + 2 * kSecond / 10),
                                               start + 2 * kSecond / 10, start + 2 * kSecond / 10);
    EXPECT_EQ(status.state, HealthState::kOk);
    EXPECT_GT(status.confidence, 0.5);
}

TEST(HealthMonitor, DegradesAndRecovers) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    monitor.update(make_sample(start), start, start);
    monitor.update(make_sample(start + 1), start + 1, start + 1);

    const Timestamp degraded_at = start + 2;
    const HealthStatus degraded =
        monitor.update(make_sample(degraded_at, true, 0.15, 0.5), degraded_at, degraded_at);
    EXPECT_EQ(degraded.state, HealthState::kDegraded);
    EXPECT_LT(degraded.confidence, 1.0);

    EXPECT_EQ(monitor.update(make_sample(degraded_at + 1), degraded_at + 1, degraded_at + 1).state,
              HealthState::kDegraded);
    EXPECT_EQ(monitor.update(make_sample(degraded_at + 2), degraded_at + 2, degraded_at + 2).state,
              HealthState::kOk);
}

TEST(HealthMonitor, LostAfterNoValidRegistration) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    monitor.update(make_sample(start), start, start);
    monitor.update(make_sample(start + kSecond / 10), start + kSecond / 10, start + kSecond / 10);

    EXPECT_EQ(
        monitor.update(make_sample(start + kSecond, false), start + kSecond, start + kSecond).state,
        HealthState::kDegraded);
    EXPECT_EQ(monitor
                  .update(make_sample(start + 2 * kSecond, false), start + 2 * kSecond,
                          start + 2 * kSecond)
                  .state,
              HealthState::kLost);

    // LOST 是粘滞的：好样本也不能静默恢复。
    EXPECT_EQ(monitor
                  .update(make_sample(start + 2 * kSecond + kSecond / 5),
                          start + 2 * kSecond + kSecond / 5, start + 2 * kSecond + kSecond / 5)
                  .state,
              HealthState::kLost);

    monitor.reset();
    EXPECT_EQ(monitor.state(), HealthState::kInitializing);
}

TEST(HealthMonitor, OdomTimeoutIsLost) {
    HealthMonitor monitor(test_config());
    const Timestamp now = kSecond;
    const HealthStatus status = monitor.update(make_sample(now), now, now - kSecond);

    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_DOUBLE_EQ(status.confidence, 0.0);
    EXPECT_EQ(status.reason, "odometry timeout");
}

TEST(HealthMonitor, SmallTimestampInversionIsTolerated) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 里程计时间戳比评估时间新 20 ms（小于 200 ms 容差）：不应误判为断流。
    EXPECT_EQ(monitor.update(make_sample(start), start, start + kSecond / 50).state,
              HealthState::kInitializing);
    EXPECT_EQ(monitor
                  .update(make_sample(start + kSecond / 10), start + kSecond / 10,
                          start + kSecond / 10 + kSecond / 50)
                  .state,
              HealthState::kOk);
}

TEST(HealthMonitor, LargeTimestampInversionStillTimesOut) {
    HealthMonitor monitor(test_config());
    const Timestamp start = kSecond;
    // 反序 500 ms 超出容差，仍按时间基准异常处理。
    const HealthStatus status = monitor.update(make_sample(start), start, start + kSecond / 2);
    EXPECT_EQ(status.state, HealthState::kLost);
    EXPECT_EQ(status.reason, "odometry timeout");
}

TEST(PoseSourceSelection, FollowsPolicy) {
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kLidar, false, true), PoseSource::kUnknown);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kExternal, true, false), PoseSource::kUnknown);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kAuto, false, true), PoseSource::kExternalOdom);
    EXPECT_EQ(select_pose_source(PoseSourcePolicy::kAuto, true, true), PoseSource::kSentryLidar);
}

} // namespace
} // namespace foray::localization
