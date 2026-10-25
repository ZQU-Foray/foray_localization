cmake_minimum_required(VERSION 3.16)
project(foray_localization_tools VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_package()
