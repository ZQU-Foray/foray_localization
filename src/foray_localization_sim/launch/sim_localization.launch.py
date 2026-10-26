# Copyright 2026 ZQU-Foray
#
# 定位层仿真入口。三种模式：
#   replay  本仓合成回放（无 Gazebo/无 GPU），CI 与日常回归用
#   gazebo  外部 Gazebo 仿真提供点云与真值，本包只注入里程计
#   external 完全由外部话题驱动（例如回放 rosbag）
#
# Gazebo 场景不在本仓：见 docs/simulation.md 的 rmu_gazebo_simulator /
# pb_rm_simulation 复现步骤。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    map_manifest = LaunchConfiguration("map_manifest")

    replay = Node(
        package="foray_localization_tools",
        executable="sim_replay.py",
        name="sim_replay",
        output="screen",
        arguments=[
            "--manifest",
            map_manifest,
            "--duration",
            LaunchConfiguration("duration"),
            "--points",
            LaunchConfiguration("points"),
            "--seed",
            LaunchConfiguration("seed"),
            "--cloud-rate",
            LaunchConfiguration("cloud_rate"),
            "--odom-rate",
            LaunchConfiguration("odom_rate"),
            "--drop-odom-after",
            LaunchConfiguration("drop_odom_after"),
            "--drop-odom-duration",
            LaunchConfiguration("drop_odom_duration"),
            "--ros",
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'replay'"])),
    )

    injector = Node(
        package="foray_localization_sim",
        executable="sim_odom_injector.py",
        name="sim_odom_injector",
        output="screen",
        parameters=[
            {
                "gt_topic": LaunchConfiguration("gt_topic"),
                "odom_topic": "/aft_mapped_to_init",
                "scale_error": LaunchConfiguration("scale_error"),
                "yaw_drift_deg_per_m": LaunchConfiguration("yaw_drift_deg_per_m"),
                "drop_after_sec": LaunchConfiguration("drop_odom_after"),
                "drop_duration_sec": LaunchConfiguration("drop_odom_duration"),
            }
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'gazebo'"])),
    )

    evaluator = Node(
        package="foray_localization_tools",
        executable="eval_trajectory.py",
        name="eval_trajectory",
        output="screen",
        arguments=[
            "--duration",
            LaunchConfiguration("eval_duration"),
            "--json",
            LaunchConfiguration("eval_json"),
            "--max-ate",
            LaunchConfiguration("max_ate"),
        ],
        condition=IfCondition(LaunchConfiguration("evaluate")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_manifest", description="先验地图 manifest.yaml"),
            DeclareLaunchArgument(
                "mode",
                default_value="replay",
                description="replay / gazebo / external",
            ),
            DeclareLaunchArgument("duration", default_value="60.0"),
            DeclareLaunchArgument("points", default_value="6000"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("cloud_rate", default_value="10.0"),
            DeclareLaunchArgument("odom_rate", default_value="50.0"),
            DeclareLaunchArgument("drop_odom_after", default_value="-1.0"),
            DeclareLaunchArgument("drop_odom_duration", default_value="1.0"),
            DeclareLaunchArgument("gt_topic", default_value="/sim/ground_truth"),
            DeclareLaunchArgument("scale_error", default_value="0.02"),
            DeclareLaunchArgument("yaw_drift_deg_per_m", default_value="0.8"),
            DeclareLaunchArgument("evaluate", default_value="false"),
            DeclareLaunchArgument("eval_duration", default_value="30.0"),
            DeclareLaunchArgument(
                "eval_json", default_value="/tmp/foray_sim_eval.json"
            ),
            DeclareLaunchArgument("max_ate", default_value="0.15"),
            LogInfo(msg=["定位仿真启动，mode=", mode]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("foray_localization_bringup"),
                            "launch",
                            "localization.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_name": "sim.yaml",
                    "map_manifest": map_manifest,
                    "start_map_server": "true",
                }.items(),
            ),
            replay,
            injector,
            evaluator,
        ]
    )
