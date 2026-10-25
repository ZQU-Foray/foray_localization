// Copyright 2026 ZQU-Foray

#include "foray/localization_ros/localization_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<foray::localization_ros::LocalizationNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
