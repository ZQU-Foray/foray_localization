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

Decision: `foray_localization_place`（2026-10-25 更名为
`foray_localization_relocalization`，见 D12）用 ONNX Runtime CPU（`FORAY_WITH_ONNX=OFF` 默认），
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

---

## D11 仿真闭环放在本仓、但不引入 Gazebo 构建依赖

Status: **superseded by D16**（2026-10-27：仿真闭环与离线工具已整体移出本仓）。
以下原文保留留痕，其中 `foray_localization_sim` / `foray_localization_tools` /
`bringup/config/sim.yaml` / `docs/simulation.md` / CI `sim-smoke` 均已不存在。

Date: 2026-10-25

Context: M5 HIL 验证需要录包/仿真回放与 ATE/RPE、DEGRADED/LOST 时序验证，
但此时无实车、机载无 GPU，且官方 Gazebo 场景（`rmu_gazebo_simulator` /
`pb_rm_simulation`）版本演进快。

Decision:
- 本仓只提供 `foray_localization_sim`（真值里程计注入 + 模式化 launch）
  与 `foray_localization_tools` 的合成回放/评估脚本；Gazebo 场景与
  Mid360 插件不入仓、不进构建依赖，按 `docs/simulation.md` 外部复现。
- 合成闭环走「先验 PCD 射线投射 → 回放 + 定位节点 → `eval_trajectory.py`
  断言 ATE/OK 占比」的路径，CI `sim-smoke` 作业无 GPU 可跑。
- 仿真阈值单独放 `bringup/config/sim.yaml`，不参与默认实车启动。

Reason:
- `.foray-layer` 依赖方向与「定位层不感知数据来源」的接口约定均不破：
  定位节点只认 `/livox/lidar`、`/aft_mapped_to_init`、TF。
- Gazebo 进仓会把渲染/物理依赖带进定位层构建，与机载无 GPU 约束冲突。
- 阈值只能初标：仿真点云无动态物体、无遮挡，且多为机械式扫描，
  实车结论仍以 `docs/simulation.md` §5 的现场标定流程为准。

Alternatives:
- 把 Gazebo 场景与插件 vendor 进本仓
- 只等实车，暂不做离车验证

Rejected:
- vendor 增加升级/审计成本，且与 `foray_ws` pin 策略冲突
- 无实车期间没有任何可重复回归，M5 会长期空转

---

## D12 包布局迁入 src/、重定位包更名

Date: 2026-10-25

Context: 组织统一「仓根 = colcon 空间根、全部包在 `src/`」的布局约定（`foray_docs`
`repository_structure.md` §7.5 待登记），并要求核心包用 `_core`、角色后缀封闭词表。
本仓 7 个包原先平铺在仓根，`foray_localization_place` 的后缀 `place` 不在词表内。

Decision:
- 7 个包全部迁入 `src/<package>/`，仓根不再有 `package.xml`；
- `foray_localization_place` → `foray_localization_relocalization`，同步
  `package.xml`、CMake `project`、命名空间 `foray::localization_relocalization`、
  头文件目录 `include/foray/localization_relocalization/`、include guard；
- 类名 `PlaceLocalizer` / `PlaceConfig` 与文件名保持（重定位手段仍是地点识别）；
- CI 新增 `dir-layout` 作业，与 `foray_platform` 使用同一套规则。

Reason:
- 与组织布局约定一致，`colcon build` / `rosdep install --from-paths src` 语义清晰；
- 「重定位」比「地点识别」更贴合对外语义（服务名已是 `Relocalize`），且 `place`
  不在封闭后缀词表内；
- 改名窗口只有现在：该包尚无下游消费方（已核对 `foray_localization_bringup`
  不 `find_package` 它），成本最低。

Alternatives:
- 保持平铺布局，只在 `foray_docs` 登记差异；
- 只迁 `src/`，不改包名。

Rejected:
- 平铺布局与组织约定冲突，后续每个仓都要写例外；
- `place` 与 `_relocalization` 并存会造成跨仓命名漂移，越晚改越贵。

## D13 Python 目标版本固定为 3.10

Date: 2026-10-25

Context: CI `test-python` 使用 `actions/setup-python` 的 3.10，与 ROS 2 humble 的
系统 Python 一致。首次真实运行中，`ruff --fix` 的 `UP017` 把
`datetime.timezone.utc` 改写成 `datetime.UTC`；后者是 Python 3.11 才引入的别名，
在 3.10 上直接 `AttributeError`，而本地开发机是 3.14，测试全绿，问题只在 CI 暴露。

Decision:
- 仓根新增 `pyproject.toml`，写 `[tool.ruff] target-version = "py310"`；
- 需要 UTC 时使用 `datetime.timezone.utc`，不得使用 `datetime.UTC`。

Reason:
- 让静态检查与自动修复也遵守运行时版本，而不是跟随开发机；
- 目标版本显式声明比逐个违规加 `noqa` 可维护。

Alternatives:
- 只在违规行加 `# noqa: UP017`；
- 把 CI 与运行时升到 3.11+。

Rejected:
- 逐行 `noqa` 无法阻止同类自动修复再次引入 3.11+ API；
- ROS 2 humble 的官方支持版本是 3.10，升级不属于本仓决策范围。

---

## D14 位姿可用性显式化：`LocalizationState.pose_valid`

Date: 2026-10-27

Context: 审计 `docs/audit-2026-10-27.md` B2 指出，`LOST` 下 `publish_state()` 照旧按
`map_to_odom_ * odom_to_base_` 合成一个「看起来很新」的位姿发出去。下游只要不盯 `state`
就会拿过期位姿闭环——正是 README「禁止静默外推」要防的事故。另一种做法是 `LOST` 期间
干脆不发 `LocalizationState`。

Decision:
- `LocalizationState` 新增 `bool pose_valid`；`to_msg_state()` 由健康状态纯映射：
  `OK` / `DEGRADED` 为 `true`，`INITIALIZING` / `LOST` 为 `false`。
- `LOST` 期间**照旧**以 `state_rate_hz` 发布状态（`confidence = 0`、TF `map → odom` 停发），
  但 `pose` 只是最后一次合成值，仅用于 rosbag 复盘。
- README「健康契约」把它写成**强制**约定：下游必须先判 `pose_valid`，`false` 时禁止消费 `pose`。

Reason:
- 停发消息会让下游失去「已经失效」这一事实，只能靠消息超时猜——诊断连续性更差，
  且 `transient_local` QoS 下「最后一次消息」语义不明确；
- 加字段是纯增量改动：消费方都在本仓、随包一起重建（ROS 2 消息类型哈希会变，
  但这是一次全仓统一改动，不像改发布时序那样要下游改控制逻辑）；
- 字段放在 `pose` 之后而非文件末尾，是为了「位姿 + 是否可用」在读定义时相邻；
- 把判据从「状态枚举的语义」压成一个布尔量，下游实现不会写错。

Alternatives:
- `LOST` 下停发 `LocalizationState`，由下游按超时判定失效；
- 只靠 `state != STATE_OK` 隐式表达（不新增字段）。

Rejected:
- 停发消息把「失效」变成不可观测事件，且与 `transient_local` 的语义纠缠；
- 隐式判据正是审计里出事的形式——文档写过「必须检查」，实现却没有单一的检查点。

## D15 参数文档由脚本看守：README ↔ `declare_parameter` ↔ 兵种配置

Status: **部分失效（D16，2026-10-27）**——`check_param_consistency.py` 与其 pytest、
CI `test-python` 作业已删除；README 参数表结构保留，三处一致改由 PR 人工核对。

Date: 2026-10-27

Context: 审计 B3 实测 README「关键参数」表有 11 个参数名在源码中不存在
（`map_manifest_path`、`registration_rate_hz`、`voxel_size`、`min_cluster_points` …），
`base_frame` 默认值也写成 `base_link`，`tree.md` 还把两个节点入口写成一个文件。
README 是本仓唯一需求来源（AGENTS.md），这类漂移会让下游照文档写配置直接启动失败。

Decision:
- 以实现为准回写 README 参数表：拆成「本地化节点参数（`foray_localization`，30 项）」
  与「地图服务节点参数（`foray_map_server`，3 项）」，**一行一个参数名、默认值抄自
  `declare_parameter`**，便于机器解析；
- 新增 `src/foray_localization_tools/scripts/check_param_consistency.py`，在 CI 的
  `test-python` 作业里断言：README 参数名集合 == `declare_parameter` 集合；兵种配置键
  ⊆ 已声明参数；兵种配置键 ⊇ 已声明参数减去 launch 注入的 `{map_manifest, verify_map_hashes}`；
- 配套 pytest（`test_param_consistency.py`）既跑真实仓库，也用 `tmp_path` 合成仓库证明
  「README 多参数 / 缺参数 / 配置键漂移」三类问题都会让脚本 exit 1。

Reason:
- 文档漂移靠人眼 Review 抓不住（11 个名字错了很久没人发现），只有可执行断言能长期生效；
- 让 README 表格结构化（一行一名）同时降低了人读成本与解析成本；
- 配置允许「多声明 launch 注入项」，所以规则写成子集/超集而不是死板相等，避免过度约束。

Alternatives:
- README 直接由脚本从源码生成（`declare_parameter` 无法提供说明文本，反而更差）；
- 只在 Review checklist 里加一条「记得同步 README」。

Rejected:
- 自动生成会丢掉「为什么是这个默认值」的解释，README 会退化成接口清单；
- checklist 无强制力，审计已经证明这类漂移会长期存活。

---

## D16 移除合成仿真与离线工具，本仓只保留核心定位算法

Date: 2026-10-27

Context: 用户要求「删除本仓库无意义的合成仿真，只保留核心定位算法」（m00001）。
盘点发现仿真不是孤立脚本而是一条闭环：`foray_localization_sim`（真值里程计注入 +
模式化 launch）、`foray_localization_tools`（合成场地生成、回放、累积建图、
ATE/RPE 评估、参数一致性校验）、`bringup/config/sim.yaml`、`docs/simulation.md`，
以及 CI 的 `sim-smoke` / `test-python` 两个作业。同时查出 `sim-smoke` 自 M7 起就是
死作业：探测条件写的是仓根 `foray_localization_sim`，而包已迁入 `src/`，
`-d` 恒为假，作业从未执行过（证据：`local/evidence-strip-sim/ci-sim-smoke-skip-proof.txt`）。

Decision:
- 删除 `src/foray_localization_sim`（4 文件）、`src/foray_localization_tools`（18 文件）、
  `src/foray_localization_bringup/config/sim.yaml`、`docs/simulation.md`、仓根
  `requirements.txt`；`src/` 只保留 `_core` / `_map` / `_ros` / `_relocalization` / `_bringup`。
- CI 只保留真实执行的 4 个作业：`dir-layout` / `lint-python` / `lint-c` / `build-ros2`；
  删除 `sim-smoke`（恒跳过）与 `test-python`（已无 `test_*.py`）。
  `dir-layout` 的豁免词表只留 `*_bringup`。
- 保留 `pyproject.toml`：`localization.launch.py` 仍是 Python，ruff/black 仍需目标版本 3.10。
- 验证责任重新划分：本仓 CI 断言「可构建 + 可执行文件安装位置 + launch 参数可解析 +
  `colcon test`」，**不**断言定位精度；车下验证改为录制包回放（README「无实车验证」）。

Reason:
- 需求来源只有 README（AGENTS.md），仿真闭环在 README 里是成章节的需求，删除必须同步
  改 README/tree/plan/decision，否则文档成为第二套过时需求。
- `sim-smoke` 恒跳过、`test-python` 无测试可跑：保留只会制造「绿灯假象」，
  与用户要求「修正被全部跳过的 CI」冲突。
- 合成仿真从未驱动过真实决策——它的产物（合成场地、回放包）与实际比赛场地无关，
  维护成本却随源码演进持续存在（审计 A3 就专门修过仿真射线投射）。

Compensation（明确记下失去的能力）:
- 失去 **参数一致性 CI 门禁**（D15）：README ↔ `declare_parameter` ↔ 兵种配置三处
  一致性改由 PR 人工核对。基线已跑过一次（30 + 3 项全等：
  `local/evidence-strip-sim/check_param_consistency.txt`），恢复工具包时必须同时恢复该作业。
- 失去 **仓内建图与地图校验工具**（`make_map.py` / `validate_map.py` /
  `accumulate_pcd.py`）：地图改在仓外制作，本仓只按 `manifest.yaml` 契约加载并校验 SHA-256。
- 失去 **离车精度回归**（ATE/RPE）：精度验收移到实车与下游仿真仓，本仓不再提供自动化精度断言。

Alternatives:
- 保留 `foray_localization_tools` 只删 `foray_localization_sim`（用户评估后否决）；
- 保留工具包但删掉其测试与 CI 作业（会留下无测试的 Python 代码，更差）；
- 只修 `sim-smoke` 的 `src/` 前缀让死作业复活（用户目标是删除仿真，不是修仿真）。

Rejected:
- 用户明确选择 C 方案：连离线工具包一起删除，只留五个核心包；
- 修补恒跳过的作业会把它从「假绿」变成「真跑合成闭环」，与删除目标相反。
