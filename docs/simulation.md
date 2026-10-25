# 仿真验证（Sim2Real 前置）

> 目标：在**没有实车**的阶段，验证定位层的位姿输出、健康状态机与重定位流程，
> 并把「数据怎么来、怎么判对错」固化成可重复的流程。
> 本文只描述**定位层**的仿真，不包含 Nav2、规划或整车控制。

## 1. 两条数据源

| 模式 | 数据源 | 需要 GPU | 用途 |
|---|---|---|---|
| `replay` | 本仓 `sim_replay.py` 从先验 PCD 射线投射 | 否 | 日常回归、CI smoke |
| `gazebo` | 外部 Gazebo 场景（官方仿真器 / pb_rm_simulation） | 通常需要（可软件渲染） | 更接近真实场地与点云模式 |
| `external` | 外部 rosbag 或自研仿真 | 否 | 复盘、现场数据回放 |

两条路都只依赖同一组接口：`/livox/lidar`（PointCloud2）、`/aft_mapped_to_init`
（Odometry）、`base_footprint → livox_frame` 静态 TF。**定位节点不感知数据来自仿真还是实车。**

## 2. 合成闭环（推荐日常使用）

```bash
# 1. 生成 12 m × 8 m 场地 + 箱体的合成 PCD
ros2 run foray_localization_tools gen_demo_field.py --output /tmp/sim/field.pcd --spacing 0.04

# 2. 建图（2D 栅格 + manifest 同源）
ros2 run foray_localization_tools make_map.py \
  --pcd /tmp/sim/field.pcd --output-dir /tmp/sim/map \
  --version sim-demo --source synthetic

# 3. 一键闭环：回放 + 定位节点 + 地图服务
ros2 launch foray_localization_sim sim_localization.launch.py \
  map_manifest:=/tmp/sim/map/manifest.yaml mode:=replay duration:=40

# 4. 另一个终端评估（ATE/RPE/OK 占比/状态迁移）
ros2 run foray_localization_tools eval_trajectory.py \
  --duration 30 --json /tmp/sim/eval.json --max-ate 0.15
```

`sim_replay.py` 默认参数下：轨迹从 `(0, 0, 0)`、航向 0 出发（与节点默认
`initial_pose` 一致），10 Hz 点云、50 Hz 里程计；`--seed` 固定随机性。

健康状态与故障注入：

```bash
# 20 s 后中断里程计 1 s：odom_timeout_sec=0.5 时应进入 LOST
ros2 launch foray_localization_sim sim_localization.launch.py \
  map_manifest:=/tmp/sim/map/manifest.yaml mode:=replay \
  duration:=40 drop_odom_after:=20 drop_odom_duration:=1.0
```

## 3. Gazebo 场景

官方场景与 Mid360 插件**不在本仓**（避免把 Gazebo 引入定位层构建依赖）：

- 官方 RM 仿真器：<https://github.com/rm-controls/rmu_gazebo_simulator>（ROS 2 humble + Gazebo Classic 11）
- 北极熊战队开源参考：<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>
  与 <https://github.com/LihanChen2004/pb_rm_simulation>（RMUC/RMUL 场地、Mid360、point_lio）

话题映射（Gazebo 侧 → 本仓接口）：

| Gazebo 侧 | 本仓接口 | 说明 |
|---|---|---|
| `/livox/lidar/pointcloud` | `/livox/lidar` | 需 remap；缺 `time` 字段不影响本层（只做配准） |
| `/livox/imu` | — | 本层不消费 IMU，由 point_lio 使用 |
| 真值 `/ground_truth`（或 p3d 插件） | `/sim/ground_truth` | 仅评估用 |
| point_lio `/aft_mapped_to_init` | 同名 | 也可用 `sim_odom_injector.py` 从真值注入漂移 |

```bash
# 外部 Gazebo 已发布点云与真值后：只注入里程计 + 定位节点
ros2 launch foray_localization_sim sim_localization.launch.py \
  map_manifest:=/path/to/map/manifest.yaml mode:=gazebo \
  gt_topic:=/sim/ground_truth
```

无 GPU 时建议：`gzserver` 无界面启动 + 点云插件用 CPU ray（`libgazebo_ros_ray_sensor`
的 `ray` 类型）；GPU ray 需要渲染后端。

## 4. 已知差异（必须写进结论）

1. 仿真点云多为**机械式扫描**，不是 MID360 非重复扫描 → 配准残差分布与实车不同，阈值只能初标。
2. 仿真点云常缺 `time` 字段 → 本层不用逐点时间；若后续接入 point_lio 建图需补。
3. 合成回放无动态物体、无遮挡、无地面反射 → 动态障碍与残差统计**不能**用仿真结论替代实车。
4. 仿真真值无轮速打滑、IMU 零偏 → `sim_odom_injector.py` 用尺度/航向漂移显式注入这些误差。

## 5. 阈值标定流程（到车后照做）

1. 现场录包：`/livox/lidar`、`/aft_mapped_to_init`、TF。
2. 同一段包回放，把 `ok_mean_residual_m` 等阈值扫一遍，记录 OK/DEGRADED/LOST 的分布。
3. 用 `eval_trajectory.py` 对真值算 ATE/RPE，填回 `bringup/config/*.yaml` 并注明来源。
4. 阈值改动必须走 `decision.md` 留痕。

## 6. CI

`.github/workflows/ci.yml` 的 `sim-smoke` 作业在 `ros:humble` 容器内构建 → 生成合成地图 →
启动回放与定位节点 → 用 `eval_trajectory.py` 断言 ATE 与 OK 占比；失败即 CI 红灯。

## 7. 离线产物

`sim_replay.py --dump-dir DIR` 会写出 `scans/cloud_<stamp>.pcd`、`poses.csv`（真值）、
`odom.csv`（漂移里程计），可用 `accumulate_pcd.py` 按真值累积成 PCD 再建图：

```bash
ros2 run foray_localization_tools sim_replay.py \
  --manifest /tmp/sim/map/manifest.yaml --duration 10 --dump-dir /tmp/sim/dump
ros2 run foray_localization_tools accumulate_pcd.py \
  --scans /tmp/sim/dump/scans --poses /tmp/sim/dump/poses.csv \
  --output /tmp/sim/accumulated.pcd
```
