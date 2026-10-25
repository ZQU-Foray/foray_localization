# L3 定位健康契约：下游只依赖本消息判断位姿是否可用。
# 状态与 confidence 必须同时发布，禁止“位姿照发、失效不说”。

# 状态机取值（与 foray::localization::HealthState 一一对应）
uint8 STATE_INITIALIZING=0
uint8 STATE_OK=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3

# 位姿来源（与 foray::localization::PoseSource 对应）
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_SENTRY_LIDAR=1
uint8 SOURCE_EXTERNAL_ODOM=2

std_msgs/Header header

# map 坐标系下的机器人位姿（base_footprint）。
geometry_msgs/PoseWithCovariance pose

# 0..1；LOST 时必须为 0。
float64 confidence

# 距最近一次有效校正的时间（秒）。
float64 age_sec

uint8 state
uint8 source

# 人类可读的原因，便于现场诊断与 rosbag 复盘。
string reason
