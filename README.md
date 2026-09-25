# foray_localization

> **L3** · owner 角色：导航

定位服务 + 地图管理 + 配准残差动态物体检测

**安全关键**——激光只做定位，构成单点故障。

---

## 快速开始

> 本仓处于**初始化状态**，尚无源码。以下流程随 P0/P1 落地逐步可用。

```bash
# 1. 拉取（仓齐备后改为从元仓 foray_ws 一键拉取）
git clone git@github.com:ZQU-Foray/foray_localization.git
cd foray_localization

# 2. 依赖安装
rosdep install -y -r --from-paths . --ignore-src --rosdistro humble

# 3. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 运行
# （待装配根就绪后补充）
```

## 依赖

| 类型 | 依赖 |
|---|---|
| **自研仓** | X2、L1、L2 |
| **第三方** | PCL · small_gicp · point_lio（pin）· Ceres |

## 接口

| 产出 | 内容 |
|---|---|
| 位姿 | 位姿 + 置信度/协方差 + 新鲜度 + **失效标志**（健康契约） |
| 地图 | 2D 栅格 + 3D 先验点云（**同源、同坐标系、同原点**）· 版本化 |
| 动态物体 | 配准残差聚类（**无标号**） |

**健康契约是本仓对 X1 的主要交付**——定位失效必须显式上报，不能被下游静默消费。

## 定位方案



内置先验地图 + 激光配准；**赛时不跑在线 SLAM**。



```

实时点云 ──配准──► 内置先验地图

                    ├──► 位姿 T          = 定位

                    └──► 配准残差 {p}    = 动态物体集合

```



保留的状态估计链：`point_lio` → `loam_interface` → `sensor_scan_generation`。

`small_gicp_relocalization` 是**定位手段**（已从「可选」升级为「核心」）。



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
