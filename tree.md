# 目录结构说明

## 布局规则

本仓根目录**就是 colcon 空间根**，所有 ROS 2 包统一放在 `src/` 下。

- 仓根不放任何 `package.xml`；`colcon build`、`rosdep install --from-paths src` 都针对 `src/`
- `src/<pkg>/` 的目录名必须等于 `package.xml` 里的 `<name>`
- 工程包（C++ 库/节点）固定骨架：`include/` + `src/` + `test/` + `CMakeLists.txt` + `package.xml`；
  `*_tools`（纯 Python 离线工具）与 `*_bringup`（装配）按需精简
- C++ 可执行小工具放 `apps/`，数据/部署资产放 `data/`；**不得出现 `tools/` 目录**
- 构建产物 `build/`、`install/`、`log/`、本地工具链与缓存 `local/` 均在 `.gitignore`，
  迁入 `src/` 后它们不再与源码同层

## 命名规则

- 核心包用 `<仓名>_core`，不与仓同名
- 角色后缀封闭词表：`_core` / `_ros` / `_map` / `_relocalization` / `_tools` / `_bringup` / `_sim`
- 重定位能力的包名是 **`foray_localization_relocalization`**
  （原 `foray_localization_place`，2026-10-25 更名，见 `decision.md` D12）
- 公共头文件目录与命名空间跟随包名：`include/foray/localization_relocalization/`、
  `namespace foray::localization_relocalization`

## 目录结构

```
.
├── README.md                      本仓定位、接口、坐标系、地图与健康契约
├── AGENTS.md                      AI 协作规范
├── plan.md                        开发计划（M0–M6）
├── decision.md                    关键工程决策（D0–D12）
├── tree.md                        本文件
├── CODEOWNERS                     Review 自动分派
├── .foray-layer                   所属层与依赖边界（CI 校验）
├── .gitignore
├── .github/
│   └── workflows/
│       └── ci.yml                 组织 CI 模板（空仓绿灯，有包构建）
│
├── docs/
│   └── simulation.md              仿真数据源、Gazebo 复现、已知差异、标定流程
│
├── src/                           全部 ROS 2 包（仓根即 colcon 空间根）
│   ├── foray_localization_core/        L3 核心，纯 C++17，无 ROS
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── cmake/foray_localization_coreConfig.cmake.in
│   │   ├── include/foray/localization/
│   │   │   ├── geometry.hpp           SE(3)/yaw/坐标变换
│   │   │   ├── health.hpp             健康状态机与位姿源仲裁
│   │   │   ├── manifest.hpp           地图 manifest 解析/校验
│   │   │   ├── pose.hpp               Pose3D / RegistrationSample 等值类型
│   │   │   └── sha256.hpp             文件/内存 sha256（地图完整性校验）
│   │   ├── src/
│   │   │   ├── geometry.cpp
│   │   │   ├── health.cpp
│   │   │   ├── manifest.cpp
│   │   │   └── sha256.cpp
│   │   └── test/
│   │       ├── test_geometry.cpp
│   │       ├── test_health.cpp
│   │       ├── test_manifest.cpp
│   │       └── test_sha256.cpp
│   │
│   ├── foray_localization_map/         L3 地图与配准，无 ROS
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── cmake/foray_localization_mapConfig.cmake.in
│   │   ├── include/foray/localization_map/
│   │   │   ├── aligner.hpp            Aligner 配准抽象
│   │   │   ├── grid_map.hpp           PGM 读取 + 2D 占据栅格查询
│   │   │   ├── icp_aligner.hpp        内置点到点 ICP 后端
│   │   │   ├── pcd_loader.hpp         PCD（ascii/binary）读取
│   │   │   ├── point_cloud.hpp        PointCloud 容器/体素降采样/变换
│   │   │   ├── prior_map.hpp          先验地图 + 2D 栅格 + manifest 绑定
│   │   │   └── residual_clusterer.hpp 残差欧氏聚类
│   │   ├── src/
│   │   │   ├── grid_map.cpp
│   │   │   ├── icp_aligner.cpp
│   │   │   ├── pcd_loader.cpp
│   │   │   ├── point_cloud.cpp
│   │   │   ├── prior_map.cpp
│   │   │   └── residual_clusterer.cpp
│   │   └── test/
│   │       ├── test_grid_map.cpp
│   │       ├── test_icp.cpp
│   │       ├── test_pcd_loader.cpp
│   │       ├── test_point_cloud.cpp
│   │       ├── test_prior_map.cpp
│   │       └── test_residual_clusterer.cpp
│   │
│   ├── foray_localization_ros/         L3 ROS 2 层（节点/TF/QoS/消息）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/
│   │   │   ├── DynamicObstacle.msg
│   │   │   ├── DynamicObstacleArray.msg
│   │   │   ├── LocalizationState.msg
│   │   │   └── MapMeta.msg
│   │   ├── srv/
│   │   │   └── Relocalize.srv
│   │   ├── include/foray/localization_ros/
│   │   │   ├── adapters.hpp           点云/里程计/外部位姿校验与转换
│   │   │   ├── conversion.hpp         ROS ↔ core/map 类型转换
│   │   │   ├── localization_node.hpp  主节点
│   │   │   └── map_server_node.hpp    地图服务节点
│   │   ├── src/
│   │   │   ├── adapters.cpp
│   │   │   ├── conversion.cpp
│   │   │   ├── localization_node.cpp
│   │   │   ├── map_server_node.cpp
│   │   │   └── nodes_main.cpp
│   │   └── test/
│   │       ├── test_adapters.cpp
│   │       └── test_conversion.cpp
│   │
│   ├── foray_localization_relocalization/  可选 ONNX Runtime CPU 重定位（默认 OFF）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/foray/localization_relocalization/place_localizer.hpp
│   │   ├── src/place_localizer.cpp
│   │   └── test/test_place_localizer.cpp
│   │
│   ├── foray_localization_tools/       离线工具（Python，无 ROS 运行时依赖）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── scripts/
│   │   │   ├── make_map.py            PCD → 2D PGM + manifest（同源投影）
│   │   │   ├── validate_map.py        字段/尺寸/相对路径/sha256 校验
│   │   │   ├── sim_common.py          纯 Python 几何/点云/射线投射/漂移模型
│   │   │   ├── gen_demo_field.py      生成 12×8 场地合成 PCD
│   │   │   ├── sim_replay.py          合成点云回放（ROS 话题 / 离线 dump）
│   │   │   ├── eval_trajectory.py     ATE/RPE/OK 占比评估，超标 exit 1
│   │   │   └── accumulate_pcd.py      按真值位姿累积点云建图
│   │   └── test/
│   │       ├── conftest.py            把 scripts/ 加入 sys.path
│   │       ├── test_sim_common.py     几何/射线投射/漂移模型/ATE-RPE 单测
│   │       ├── test_eval_trajectory.py 评估汇总与离线 CLI 退出码单测
│   │       ├── test_sim_assets.py     launch 语法、sim.yaml 键、注入器漂移单测
│   │       └── test_offline_pipeline.py 合成→建图→回放→累积 端到端
│   │
│   ├── foray_localization_sim/         仿真验证（不含 Gazebo 依赖）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── scripts/
│   │   │   └── sim_odom_injector.py   真值 → 漂移里程计，支持断流注入
│   │   └── launch/
│   │       └── sim_localization.launch.py  replay / gazebo / external 三模式
│   │
│   └── foray_localization_bringup/     兵种装配（2027 3v3 用步兵/哨兵）
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   ├── sentry.yaml
│       │   ├── infantry.yaml
│       │   ├── heavy.yaml             装甲兵种预留：2027 3v3 阵容不使用（步兵×2+哨兵×1）
│       │   └── sim.yaml               仿真专用阈值（不参与默认启动）
│       └── launch/
│           └── localization.launch.py   由 config_name 选择兵种参数
│
├── local/                          本地工具链/地图缓存（gitignore，不入库）
└── log/                            运行日志（gitignore，不入库）
```

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- `local/`（本地工具链/标定/地图缓存，已在 `.gitignore`）
- 临时文件、缓存文件、日志文件
