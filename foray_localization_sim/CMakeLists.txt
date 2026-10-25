cmake_minimum_required(VERSION 3.16)
project(foray_localization_sim VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(
    PROGRAMS scripts/sim_odom_injector.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
