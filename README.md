# foray_localization

> **L3** · owner 角色：导航

定位服务 + 地图管理 + 配准残差动态物体检测

**安全关键**——激光只做定位，构成单点故障。

---

## 快速开始

```bash
# 1. 拉取（仓齐备后改为从元仓 foray_ws 一键拉取）
git clone git@github.com:ZQU-Foray/foray_localization.git
cd foray_localization

# 2. 依赖安装（small_gicp 需按 foray_ws 的 pin 单独构建，见「依赖」）
rosdep install -y -r --from-paths . --ignore-src --rosdistro humble

# 3. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 运行（哨兵；步兵/重装见 bringup 的对应 launch）
source install/setup.bash
ros2 launch foray_localization_bringup sentry.launch.py \
  map_manifest:=/path/to/map/manifest.yaml
```

## 定位方案

内置**先验地图** + 激光配准；**赛时不跑在线 SLAM**。

```
MID360 点云 ──► point_lio（仅里程计）──► T_odom_lidar
      │                                      │
      └──► 5 Hz GICP ──► 先验点云地图        │
                    │                       │
                    ├──► T_map_lidar ───────┴──► T_map_odom ──► TF（20 Hz）
                    │                                   │
                    └──► 配准残差 {p} ──► 聚类 ──► 动态物体（无标号）
```

- **里程计**：`point_lio` 只提供 `odom → lidar`，不参与地图更新；赛时不做 SLAM。
- **配准**：`foray_localization_map` 以先验 PCD 地图为目标做 GICP（默认 PCL GICP，5 Hz），
  输出 `T_map_lidar`、fitness、内点率与残差点。
- **融合**：`foray_localization_ros` 由 `T_map_lidar` 与 `T_odom_lidar` 合成
  `map → odom`，以 20 Hz 发布；`LocalizationState` 以 50 Hz 发布。
- **健康**：状态机 `INITIALIZING → OK ⇄ DEGRADED → LOST`，失效显式上报，**禁止静默外推**。
- **重定位**：固定出生点初值；丢失后调用 `/localization/relocalize` 服务注入初值。
  自动全局重定位（ONNX Scan Context）为可选 P2，默认关闭。
- **动态物体**：配准残差聚类，输出质心/尺寸/点数/置信度，**无类别标号**；
  与「地图过期」通过持续时间、允许动态区域标注区分。

## Package 布局

| package | 职责 | 是否依赖 ROS |
|---|---|---|
| `foray_localization_core` | 位姿/健康状态机/位姿源仲裁/地图 manifest/几何，纯函数 | 否 |
| `foray_localization_map` | PCD 先验地图、2D 栅格、GICP 后端、残差聚类 | 否 |
| `foray_localization_ros` | 节点、TF、QoS、消息/服务、map_server、适配器 | 是 |
| `foray_localization_place` | 可选 ONNX Runtime（CPU）全局重定位，默认 OFF | 可选 |
| `foray_localization_tools` | 建图、2D 投影、`validate_map`、`bench_lio` | 否（Python） |
| `foray_localization_bringup` | 三兵种 launch / params | 是 |

## 接口

### 话题

| 方向 | 名称 | 类型 | QoS / 频率 |
|---|---|---|---|
| 输入 | `/aft_mapped_to_init` | `nav_msgs/Odometry` | sensor_data / 100–200 Hz |
| 输入 | `/livox/lidar` | `sensor_msgs/PointCloud2` | sensor_data / 10 Hz |
| 输入 | `/localization/external_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 步兵/重装外部位姿，50 Hz |
| 输出 | `/localization/state` | `foray_localization_ros/LocalizationState` | reliable, transient_local / 50 Hz |
| 输出 | `/localization/map_meta` | `foray_localization_ros/MapMeta` | transient_local / 1 Hz |
| 输出 | `/localization/dynamic_obstacles` | `foray_localization_ros/DynamicObstacleArray` | reliable / 10 Hz |

### 服务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/localization/relocalize` | `foray_localization_ros/Relocalize` | 注入 `map → base_link` 初值，状态机回到 `INITIALIZING` 直到收敛 |

### TF

| 变换 | 发布者 | 频率 |
|---|---|---|
| `map → odom` | `foray_localization_ros` | 20 Hz |
| `odom → lidar` | `point_lio` | 100–200 Hz |
| `base_link → lidar` | URDF / `robot_state_publisher`（静态） | — |

### 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_manifest_path` | 必填 | 地图 manifest 路径，启动时强校验 |
| `pose_source` | `auto` | `auto` / `lidar` / `external`；步兵/重装用 `external` |
| `map_frame` / `odom_frame` / `base_frame` / `lidar_frame` | `map` / `odom` / `base_link` / `livox_frame` | 坐标系名 |
| `registration_rate_hz` | `5.0` | GICP 频率上限 |
| `state_rate_hz` | `50.0` | 状态发布频率 |
| `tf_rate_hz` | `20.0` | `map → odom` 发布频率 |
| `voxel_size` | `0.10` | 实时点云降采样（m） |
| `map_voxel_size` | `0.25` | 先验地图降采样（m） |
| `max_correspondence_distance` | `1.0` | GICP 最大对应距离（m） |
| `init_max_correspondence_distance` | `3.0` | 首帧/重定位最大对应距离（m） |
| `ok_mean_residual` | `0.10` | 进入 `OK` 的平均残差阈值（m） |
| `degraded_mean_residual` | `0.20` | 进入 `LOST` 前的平均残差上限（m） |
| `ok_inlier_ratio` | `0.55` | 进入 `OK` 的内点率 |
| `degraded_inlier_ratio` | `0.35` | `DEGRADED` 的内点率下限 |
| `lost_timeout_sec` | `1.0` | 无有效配准多久后 `LOST` |
| `residual_threshold` | `0.20` | 残差判定距离（m） |
| `cluster_tolerance` | `0.30` | 欧氏聚类容差（m） |
| `min_cluster_points` | `10` | 最小簇点数 |
| `initial_pose` | `[0, 0, 0]` | 固定出生点 `[x, y, yaw]`（map 系） |

> 阈值均以 2027 赛季 12 m × 8 m 场地与 MID360 噪声水平为初值，
> 赛前用录包标定后写入 `bringup/config/*.yaml`，**魔法数字必须注明来源**。

## 地图

3D 先验点云（PCD）与 2D 栅格（PGM/YAML）由**同一次建图**投影得到，
**同源、同坐标系、同原点**。地图不入 git，按 manifest 版本化分发。

```yaml
# manifest.yaml 示例
version: "2027.1"
frame_id: "map"
origin: [-6.0, -4.0, 0.0]   # 2D 栅格左下角在 map 系的位置 [x, y, yaw]
resolution: 0.05
width: 240
height: 160
pcd: "map.pcd"
grid: "map.pgm"
pcd_sha256: "<sha256>"
grid_sha256: "<sha256>"
created_at: "2026-10-25"
source: "RMUL 2027 field survey"
```

坐标系约定：`map` 原点为**场地中心**，`+X` 红方 → 蓝方，`+Y` 红方左侧，`+Z` 向上。
2D 栅格的 `origin` 是左下角像素在 `map` 系的位置，`resolution` 单位为 m/px。

## 健康契约

| 状态 | 条件（持续去抖） | 下游行为 |
|---|---|---|
| `INITIALIZING` | 启动/重定位后，尚未连续 3 帧有效配准 | 不得用于决策，等待 |
| `OK` | 平均残差 ≤ `ok_mean_residual` 且内点率 ≥ `ok_inlier_ratio` | 正常消费 |
| `DEGRADED` | 残差或内点率越界，但仍在 `degraded_*` 内；≤ 200 ms 公告 | 降级消费，规划减速 |
| `LOST` | 连续 `lost_timeout_sec` 无有效配准，或 odom 断流 > 0.5 s | 停止消费，显式失效 |

`LocalizationState` 同时携带 `confidence`、`age`（距上次有效配准）、`fitness`、
`inlier_ratio`、`residual_points/clusters`、`map_version`。**失效标志不可被下游静默忽略。**

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2（foray_interfaces）· L1（foray_platform）· L2 |
| **第三方** | Eigen3 · yaml-cpp · PCL（GICP，可选但推荐）· small_gicp（可选替换后端）· point_lio（pin）· ONNX Runtime（可选，默认 OFF） |

- `point_lio` 以独立进程运行，按 `foray_ws` 的 pin 构建；本仓只消费其话题。
- `small_gicp` 未安装时自动回退到内置点到点 ICP（精度较低，仅用于调试/降级）。
- `ONNX Runtime` 仅 `foray_localization_place` 使用，`FORAY_WITH_ONNX=ON` 时编译。

## 纪律

- **场地坐标系原点全队统一**，对齐规则定义的场地原点（**不是**机器人启动位置）——这是机间共享坐标的前提
- 2D 栅格与 3D 点云**必须同源同坐标系**，否则态势融合无法工作
- 需区分「**地图过期**」与「**动态物体**」：残差聚类 + 持续时间判定 + 先验「允许动态区域」标注
- 已删除：`terrain_analysis` / `terrain_analysis_ext` / `IntensityVoxelLayer` / `pointcloud_to_laserscan` / 在线 SLAM

## 上下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces` · `foray_platform` |
| **下游**（谁依赖本仓） | `foray_vision` · `foray_navigation` · `foray_decision` · `foray_robots` · `foray_ws` |

## 参考

- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md)
- [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
