// Copyright 2026 ZQU-Foray

#include "foray/localization_place/place_localizer.hpp"

#include <gtest/gtest.h>

namespace {

TEST(PlaceLocalizerTest, DisabledByDefault) {
    foray::localization_place::PlaceLocalizer localizer;
    foray::localization_map::PointCloud cloud;
    cloud.points.emplace_back(0.0F, 0.0F, 0.0F);
    foray::localization_map::PriorMap map;
    map.cloud = cloud;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST(PlaceLocalizerTest, EnabledWithoutOnnxReportsUnavailable) {
    foray::localization_place::PlaceConfig config;
    config.enabled = true;
    config.model_path = "/nonexistent/model.onnx";
    foray::localization_place::PlaceLocalizer localizer(config);

    foray::localization_map::PointCloud cloud;
    cloud.points.emplace_back(0.0F, 0.0F, 0.0F);
    foray::localization_map::PriorMap map;
    map.cloud = cloud;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.error.empty());
}

TEST(PlaceLocalizerTest, EmptyInputRejected) {
    foray::localization_place::PlaceConfig config;
    config.enabled = true;
    foray::localization_place::PlaceLocalizer localizer(config);
    foray::localization_map::PointCloud cloud;
    foray::localization_map::PriorMap map;

    const auto result = localizer.estimate(cloud, map);
    EXPECT_FALSE(result.ok);
}

} // namespace
