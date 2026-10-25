cmake_minimum_required(VERSION 3.16)
project(foray_localization_tools VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

# 供 ros2 run / launch Node 直接调用（建图、校验、仿真回放与评估）
install(
    PROGRAMS
    scripts/make_map.py
    scripts/validate_map.py
    scripts/sim_replay.py
    scripts/eval_trajectory.py
    scripts/gen_demo_field.py
    scripts/accumulate_pcd.py
    DESTINATION lib/${PROJECT_NAME}
)

# 纯 Python 模块：上述脚本在 install 空间按同目录 import sim_common
install(
    FILES
    scripts/sim_common.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
