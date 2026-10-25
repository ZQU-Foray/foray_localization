# 目录结构说明

## 目录结构

```
.
├── README.md                      本仓定位、接口、坐标系、地图与健康契约
├── AGENTS.md                      AI 协作规范
├── plan.md                        开发计划（M0–M6）
├── decision.md                    关键工程决策（D0–D10）
├── tree.md                        本文件
├── CODEOWNERS                     Review 自动分派
├── .foray-layer                   所属层与依赖边界（CI 校验）
├── .gitignore
├── .github/
│   └── workflows/
│       └── ci.yml                 组织 CI 模板（空仓绿灯，有包构建）
│
├── foray_localization_core/       L3 核心，纯 C++17，无 ROS
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── cmake/foray_localization_coreConfig.cmake.in
│   ├── include/foray/localization/
│   │   ├── geometry.hpp           SE(3)/yaw/坐标变换
│   │   ├── health.hpp             健康状态机与位姿源仲裁
│   │   ├── manifest.hpp           地图 manifest 解析/校验
│   │   ├── pose.hpp               Pose3D / RegistrationSample 等值类型
│   │   └── sha256.hpp             文件/内存 sha256（地图完整性校验）
│   ├── src/
│   │   ├── geometry.cpp
│   │   ├── health.cpp
│   │   ├── manifest.cpp
│   │   └── sha256.cpp
│   └── test/
│       ├── test_geometry.cpp
│       ├── test_health.cpp
│       ├── test_manifest.cpp
│       └── test_sha256.cpp
│
├── foray_localization_map/        L3 地图与配准，无 ROS
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── cmake/foray_localization_mapConfig.cmake.in
│   ├── include/foray/localization_map/
│   │   ├── aligner.hpp            Aligner 配准抽象
│   │   ├── grid_map.hpp           PGM 读取 + 2D 占据栅格查询
│   │   ├── icp_aligner.hpp        内置点到点 ICP 后端
│   │   ├── pcd_loader.hpp         PCD（ascii/binary）读取
│   │   ├── point_cloud.hpp        PointCloud 容器/体素降采样/变换
│   │   ├── prior_map.hpp          先验地图 + 2D 栅格 + manifest 绑定
│   │   └── residual_clusterer.hpp 残差欧氏聚类
│   ├── src/
│   │   ├── grid_map.cpp
│   │   ├── icp_aligner.cpp
│   │   ├── pcd_loader.cpp
│   │   ├── point_cloud.cpp
│   │   ├── prior_map.cpp
│   │   └── residual_clusterer.cpp
│   └── test/
│       ├── test_grid_map.cpp
│       ├── test_icp.cpp
│       ├── test_pcd_loader.cpp
│       ├── test_point_cloud.cpp
│       ├── test_prior_map.cpp
│       └── test_residual_clusterer.cpp
│
├── foray_localization_ros/        L3 ROS 2 层（节点/TF/QoS/消息）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   ├── DynamicObstacle.msg
│   │   ├── DynamicObstacleArray.msg
│   │   ├── LocalizationState.msg
│   │   └── MapMeta.msg
│   ├── srv/
│   │   └── Relocalize.srv
│   ├── include/foray/localization_ros/
│   │   ├── adapters.hpp           点云/里程计/外部位姿校验与转换
│   │   ├── conversion.hpp         ROS ↔ core/map 类型转换
│   │   ├── localization_node.hpp  主节点
│   │   └── map_server_node.hpp    地图服务节点
│   ├── src/
│   │   ├── adapters.cpp
│   │   ├── conversion.cpp
│   │   ├── localization_node.cpp
│   │   ├── map_server_node.cpp
│   │   └── nodes_main.cpp
│   └── test/
│       ├── test_adapters.cpp
│       └── test_conversion.cpp
│
├── foray_localization_place/      可选 ONNX Runtime CPU 重定位（默认 OFF）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/foray/localization_place/place_localizer.hpp
│   ├── src/place_localizer.cpp
│   └── test/test_place_localizer.cpp
│
├── foray_localization_tools/      离线工具（Python，无 ROS 运行时依赖）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── scripts/
│       ├── make_map.py            PCD → 2D PGM + manifest（同源投影）
│       └── validate_map.py        字段/尺寸/相对路径/sha256 校验
│
└── foray_localization_bringup/    三兵种装配
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── sentry.yaml
    │   ├── infantry.yaml
    │   └── heavy.yaml
    └── launch/
        └── localization.launch.py   由 config_name 选择兵种参数
```

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- `local/`（本地工具链/标定/地图缓存，已在 `.gitignore`）
- 临时文件、缓存文件、日志文件
