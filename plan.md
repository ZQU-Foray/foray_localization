# 开发计划

## 任务列表

- [x] **M0 骨架与文档**：6 个 package 目录/CMake/package.xml、CI 接入、README/tree/decision 对齐
- [x] **M1 里程计与地图管线**：point_lio 话题接入、`build_map`/`validate_map`、manifest 格式冻结
- [x] **M2 核心与配准**：健康状态机、位姿源仲裁、manifest 校验、GICP（PCL 后端 + ICP 兜底）、残差点
- [x] **M3 ROS 层**：LocalizationState/MapMeta/DynamicObstacle/Relocalize 消息、主节点、map_server、TF/QoS
- [x] **M4 动态物体与重定位**：残差聚类发布、Relocalize 服务与状态机联动、允许动态区域
- [ ] **M5 HIL 验证**：录包/仿真回放、ATE/RPE、DEGRADED/LOST 时序验证
  - [x] **M5a 离车仿真闭环**：合成点云回放、真值里程计注入、ATE/RPE 评估、CI smoke（`docs/simulation.md`）
  - [ ] **M5b Gazebo 联调**：官方 `rmu_gazebo_simulator` / `pb_rm_simulation` 场景接入
  - [ ] **M5c 实车 bag 验证**：现场录包后复算阈值并写回 config
- [ ] **M6 调参与交付**：2027 场地地图、阈值标定、运维文档、兵种 launch 验收（3v3：步兵/哨兵）
- [x] **M7 布局与命名规范化**：7 个包迁入 `src/`（仓根即 colcon 空间根）、
  `foray_localization_place` → `foray_localization_relocalization`、CI `dir-layout` 作业、
  文档与配置同步（见 `decision.md` D12）
- [ ] **P2 可选**：ONNX Runtime CPU 全局重定位（默认 OFF）

## 当前 Agent State

DELIVERED：7 个 package 全部位于 `src/`（仓根即 colcon 空间根），重定位包已更名为
`foray_localization_relocalization`。离车仿真闭环已落地：合成回放 → 定位节点 →
`eval_trajectory.py` 断言 ATE/OK 占比，CI `sim-smoke` / `dir-layout` 覆盖。
M5b（Gazebo 场景）/ M5c（实车 bag）仍待外部条件。
已知限制：本机无 ROS 2/colcon，`colcon build` 与 C++ 测试回归由 CI 执行。

## Before Snapshot

commit hash:   （首次提交）
branch:        dev
modified files: 6 个 package 新文件（见 git status）
risk level:    L1

## 模糊点与待确认项

- `foray_interfaces` 的 0.2.0 消息尚未落地；首版消息暂放 `foray_localization_ros`，迁移计划见 `decision.md` D8
- `small_gicp` 尚未在 `foray_ws` pin；首版用 PCL GICP + 内置 ICP 兜底，见 `decision.md` D9
- `bringup/config/heavy.yaml` 对应装甲兵种，不在 2027 3v3 阵容（步兵×2 + 哨兵×1，
  见 `foray_docs/algorithm_structure.md` §12）；保留为预留配置，不参与验收
- 2027 场地地图需赛前场地窗口实测；当前以仿真/录包验证（M5 未完成）
- 自动全局重定位（ONNX）为 P2，默认关闭；许可证决策不在本期范围

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [ ] 本仓能独立 `colcon build`（无代码时跳过）
- [ ] CI 绿灯
- [ ] 每个新增模块带独立可执行测试
- [ ] 录包/仿真回放 ATE ≤ 5 cm、航向 ≤ 2°（相对起点）
- [ ] 定位初始化 ≤ 5 s；`LocalizationState` ≥ 50 Hz；GICP 5 Hz 且 p99 ≤ 40 ms
- [ ] 配准失效 ≤ 200 ms 输出 `DEGRADED`、≤ 1 s 输出 `LOST`，无静默外推
- [ ] 同源地图 2D/3D 一致率 ≥ 95%，manifest 强校验
- [ ] 动态物体检出率 ≥ 90%（标注集）
- [ ] 兵种 launch 可分别启动（3v3：步兵/哨兵），参数错误有明确报错
