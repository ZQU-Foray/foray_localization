cmake_minimum_required(VERSION 3.16)
project(foray_localization_map VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(foray_localization_core REQUIRED)
find_package(ament_cmake QUIET)

add_library(${PROJECT_NAME} SHARED
    src/point_cloud.cpp
    src/pcd_loader.cpp
    src/grid_map.cpp
    src/prior_map.cpp
    src/icp_aligner.cpp
    src/residual_clusterer.cpp
)
add_library(foray_localization::map ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME map)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# 跨包头文件依赖显式固定，避免 colcon 增量安装时 ament 变量未及时刷新。
if(DEFINED foray_localization_core_DIR)
    get_filename_component(_foray_core_prefix "${foray_localization_core_DIR}/../../.." ABSOLUTE)
    target_include_directories(${PROJECT_NAME} PUBLIC "${_foray_core_prefix}/include")
endif()

if(ament_cmake_FOUND)
    ament_target_dependencies(${PROJECT_NAME} foray_localization_core)
    target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen foray_localization::core)
endif()

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
    EXPORT export_${PROJECT_NAME}
    NAMESPACE foray_localization::
    DESTINATION share/${PROJECT_NAME}/cmake
)

if(NOT ament_cmake_FOUND)
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)
endif()

enable_testing()

if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)

    function(foray_add_test name source)
        ament_add_gtest(${name} ${source})
        target_link_libraries(${name} ${PROJECT_NAME})
    endfunction()
else()
    find_package(GTest REQUIRED)

    function(foray_add_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
        add_test(NAME ${name} COMMAND ${name})
    endfunction()
endif()

foray_add_test(test_point_cloud test/test_point_cloud.cpp)
foray_add_test(test_pcd_loader test/test_pcd_loader.cpp)
foray_add_test(test_grid_map test/test_grid_map.cpp)
foray_add_test(test_prior_map test/test_prior_map.cpp)
foray_add_test(test_icp test/test_icp.cpp)
foray_add_test(test_residual_clusterer test/test_residual_clusterer.cpp)

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(Eigen3 foray_localization_core)
    ament_package()
endif()
