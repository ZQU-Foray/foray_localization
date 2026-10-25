# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-02-19

Context: 本仓从 0 初始化。需要确定它在整体架构中的位置与依赖边界。

Decision: 本仓定位为 **L3** 层，owner 角色为「导航」，
依赖层级 X2、L1、L2。

Reason:
- 切分判据 C1–C6 见 `foray_docs/repository_structure.md` §1
- 粒度结论：**一个仓 = 一个团队角色 + 一个变更率档位**
- **安全关键**——激光只做定位，构成单点故障

Alternatives:
- 并入相邻仓（减少仓库数量，但违反团队边界或变更率差异）

Rejected:
- 按 ROS 2 package 粒度切仓 —— 会导致版本矩阵地狱
- 按分层机械切仓 —— 会切断团队边界（Conway）

---

## D1 定位服务对位姿源解耦

Date: 2026-10-25

Context: 3v3 中哨兵装 MID360，步兵/重装无雷达，由里程计/视觉提供位姿；
后续还要接 UWB、视觉位姿。

Decision: 核心健康契约 `LocalizationState` 与位姿源无关，统一以
`source`（`SENTRY_LIDAR` / `EXTERNAL_ODOM`）区分；`foray_localization_core`
不依赖任何 ROS 类型。

Reason:
- 下游（导航/决策/视觉）只消费统一契约，不关心来源
- 步兵/重装可复用同一状态机与新鲜度/失效逻辑

Alternatives:
- 为每兵种单独定义消息/节点

Rejected:
- 让下游按话题名猜来源——会把兵种差异泄漏到所有消费者

---

## D2 point_lio 只做里程计，配准独立成层

Date: 2026-10-25

Context: point_lio 支持先验 PCD，但每帧在 LIO 内更新地图会与「固定先验地图」目标冲突，
且赛时不允许在线 SLAM。

Decision: `point_lio` 只输出 `odom → lidar`；`foray_localization_map` 独立做
先验地图 GICP（5 Hz），`foray_localization_ros` 合成 `map → odom`。

Reason:
- 先验地图保持只读，避免漂移污染与资源竞争
- 配准频率与 LIO 解耦，可单独限流/降级
- 失效时能明确区分「LIO 漂移」与「配准失败」

Alternatives:
- point_lio 内置先验 PCD 直接输出 map 位姿
- LIO-SAM / FAST-LIO2 整栈替换

Rejected:
- 内置先验 PCD 在长走廊/大场景退化，且无法独立观测配准质量
- 整栈替换会牵动点云预处理与 IMU 标定，超出本期

---

## D3 首版固定出生点 + 手动 Relocalize，不做自动全局重定位

Date: 2026-10-25

Context: 2027 3v3 场地 12 m × 8 m，出生点固定；自动全局重定位需要地点识别/描述子，
增加启动时间与误匹配风险。

Decision: 启动用 `initial_pose` 固定初值；丢失后通过
`/localization/relocalize` 服务注入初值并回到 `INITIALIZING`。
自动全局重定位（ONNX Scan Context）为可选 P2，默认关闭。

Reason:
- 赛时确定性优先；固定出生点可把「初始化」变成可复现步骤
- 手动重定位责任明确，便于操作手/裁判流程配合

Alternatives:
- 启动即自动全局重定位
- 依赖 UWB/视觉给初值

Rejected:
- 自动重定位误匹配会静默给出错误位姿，违背安全关键原则
- UWB/视觉初值作为增强，不作为首版依赖

---

## D4 地图资产 = 3D PCD + 2D 栅格 + manifest

Date: 2026-10-25

Context: 下游态势融合需要 2D 栅格，配准需要 3D 点云；两者若分别建图会坐标系不一致。

Decision: 同一次建图投影出 2D 栅格；manifest 记录版本、`frame_id`、`origin`、
`resolution`、`width/height`、PCD/栅格 sha256、建图时间与来源。
PCD/PGM 不入 git，按 manifest 分发。

Reason:
- 保证「同源、同坐标系、同原点」
- sha256 可在启动时强校验，防止地图/参数错配

Alternatives:
- 2D 与 3D 分别建图
- 地图直接入 git

Rejected:
- 分别建图会产生不可检测的错位
- 大文件入 git 违反仓库约定

---

## D5 场地坐标原点 = 场地中心

Date: 2026-10-25

Context: 规则未定义坐标原点；各队常以出生点/机器人启动位置为原点，导致机间坐标不共享。

Decision: `map` 原点为**场地中心**，`+X` 红方 → 蓝方，`+Y` 红方左侧，`+Z` 向上；
2D 栅格 `origin` 为左下角像素在 `map` 系的位置。

Reason:
- 机间共享坐标的前提；与规则场地尺寸（12 m × 8 m）对齐
- 红蓝对称，便于镜像/换边

Alternatives:
- 以红方出生点为原点
- 以机器人启动位置为原点

Rejected:
- 出生点/启动位置因兵种和操作而异，无法作为全队契约

---

## D6 动态物体 = 配准残差聚类，无标号

Date: 2026-10-25

Context: 赛时动态物体包括敌方机器人、裁判系统道具等；语义分割需要训练数据与算力。

Decision: 对配准后残差点做体素/欧氏聚类，输出质心、尺寸、点数、置信度，
**不输出类别**；用持续时间与「允许动态区域」标注抑制地图过期/反光噪声。

Reason:
- 无需训练、对任意障碍有效，CPU 可实时
- 无标号避免下游误用类别，由决策层结合视觉再分类

Alternatives:
- 训练点云语义分割
- 直接减去队友广播位姿

Rejected:
- 语义分割数据/算力成本高，且赛场外观域差异大
- 队友位姿只能去掉己方，不能覆盖敌方/道具

---

## D7 输入用标准 ROS 话题，平台 HAL 由 X3 桥接

Date: 2026-10-25

Context: `foray_platform` 目前只有 HAL C++ 库，没有 ROS 节点；本仓需要可独立构建。

Decision: 本仓订阅标准 `sensor_msgs/PointCloud2` / `nav_msgs/Odometry`；
平台 HAL → ROS 的桥接由 X3（foray_robots/装配根）提供。`scan_adapter` 负责
frame/stamp/字段校验，不直接链接 HAL。

Reason:
- 本仓可独立 colcon build/test，不被平台进度阻塞
- 标准消息便于录包回放与仿真

Alternatives:
- 直接链接 `foray_platform` HAL 并自带驱动节点

Rejected:
- 会把驱动/设备生命周期耦合进定位层，违反分层

---

## D8 消息过渡期放在 foray_localization_ros，后续迁 X1

Date: 2026-10-25

Context: `foray_interfaces` 当前无 `package.xml`/rosidl，且不在本仓可写范围；
定位层开发不能因此停摆。

Decision: 首版 `LocalizationState` / `MapMeta` / `DynamicObstacle(Array)` /
`Relocalize` 定义在 `foray_localization_ros/msg|srv`，字段与 X1 0.2.0 对齐；
X1 落地后迁移并保留兼容别名。

Reason:
- 解除跨仓阻塞，先打通技术链
- 字段设计按契约评审，迁移成本可控

Alternatives:
- 等 X1 提供消息后再开发
- 只用标准消息 + diagnostic_msgs 承载健康字段

Rejected:
- 阻塞关键路径
- 标准消息无法表达状态/置信度/age 的强类型契约

---

## D9 首版 GICP 后端用 PCL，内置 ICP 兜底

Date: 2026-10-25

Context: `small_gicp` 未在 `foray_ws` pin，且无 rosdep key；PCL 在 ROS 2 Humble
环境稳定可用，CI 可直接安装。

Decision: `foray_localization_map` 定义 `Aligner` 接口；
本次实现 `IcpAligner`（内置点到点 ICP：角点网格最近邻 + SVD 位姿更新，
带内点率/平均残差/fitness 评估）作为默认后端；
`small_gicp` 与 PCL GICP 作为后续可插拔后端，通过同一接口接入。

Revision (2026-10-25): 实现期本机与 CI 构建环境均无 PCL，`FORAY_WITH_PCL`
未落地。为避免未验证代码进入关键路径，先交付可测试的内置 ICP，并保留
`Aligner` 抽象；赛前实测若需要 GICP 精度，再接 PCL/small_gicp 后端。

Reason:
- 保持 CI 与本地可构建，不把未 pin 的第三方塞进关键路径
- 接口不变，后续换 small_gicp/PCL 不影响上层
- 内置 ICP 可被合成点云回归测试覆盖，当前测试全绿

Alternatives:
- 立即 vendor small_gicp 源码
- 只用内置 ICP

Rejected:
- 第三方源码入库增加升级/审计成本，且与 foray_ws pin 策略冲突
- 内置 ICP 精度不足，不能作为赛场默认

---

## D10 ONNX 重定位可选、默认 OFF；许可证不在本期范围

Date: 2026-10-25

Context: 用户要求计划考虑纯 CPU ONNX Runtime；许可证问题用户明确暂不考虑。

Decision: `foray_localization_place` 用 ONNX Runtime CPU（`FORAY_WITH_ONNX=OFF` 默认），
不引入 CUDA/TensorRT EP；定位主链不依赖 ONNX。许可证决策移出本期，
`package.xml` 的 `<license>` 暂保持 `TODO`，待组织统一。

Reason:
- 主链确定性优先，可选能力不阻塞交付
- 纯 CPU 满足机载无 GPU 约束

Alternatives:
- 默认启用 ONNX 全局重定位
- 本期同时定许可证

Rejected:
- 默认启用会增加启动时间与误匹配面
- 许可证与代码交付解耦，避免阻塞
