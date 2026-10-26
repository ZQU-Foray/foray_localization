#!/usr/bin/env python3
"""仿真/回放用里程计注入器：读取真值位姿，输出带漂移的 odom。

用途：没有 point_lio 时，用 Gazebo 真值或回放真值驱动定位层，并注入
尺度误差、航向漂移、噪声与断流，验证健康状态机与重定位流程。
"""

import math
import random


def wrap_angle(angle):
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class OdomDrift:
    """按位移比例累积尺度误差与航向漂移。"""

    def __init__(self, node):
        self.scale_error = node.declare_parameter("scale_error", 0.02).value
        self.yaw_drift = math.radians(
            node.declare_parameter("yaw_drift_deg_per_m", 0.8).value
        )
        self.yaw_noise = math.radians(
            node.declare_parameter("yaw_noise_deg", 0.05).value
        )
        self.pos_noise = node.declare_parameter("pos_noise_m", 0.005).value
        self.rng = random.Random(node.declare_parameter("seed", 0).value)
        self.last = None
        self.pose = (0.0, 0.0, 0.0)

    def update(self, x, y, yaw):
        if self.last is None:
            self.last = (x, y, yaw)
            self.pose = (x, y, yaw)
            return self.pose
        lx, ly, lyaw = self.last
        dx, dy = x - lx, y - ly
        distance = math.hypot(dx, dy)
        oyaw = wrap_angle(yaw - lyaw + self.yaw_drift * distance)
        ox = dx * (1.0 + self.scale_error) - dy * oyaw * 0.5
        oy = dy * (1.0 + self.scale_error) + dx * oyaw * 0.5
        self.pose = (
            self.pose[0] + ox + self.rng.gauss(0.0, self.pos_noise),
            self.pose[1] + oy + self.rng.gauss(0.0, self.pos_noise),
            wrap_angle(self.pose[2] + oyaw + self.rng.gauss(0.0, self.yaw_noise)),
        )
        self.last = (x, y, yaw)
        return self.pose


def main():
    import rclpy
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("sim_odom_injector")
    gt_topic = node.declare_parameter("gt_topic", "/sim/ground_truth").value
    odom_topic = node.declare_parameter("odom_topic", "/aft_mapped_to_init").value
    odom_frame = node.declare_parameter("odom_frame", "odom").value
    base_frame = node.declare_parameter("base_frame", "base_footprint").value
    drop_after = node.declare_parameter("drop_after_sec", -1.0).value
    drop_duration = node.declare_parameter("drop_duration_sec", 1.0).value
    drift = OdomDrift(node)
    publisher = node.create_publisher(Odometry, odom_topic, 20)
    state = {"t0": None, "last": None}

    def on_ground_truth(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if state["t0"] is None:
            state["t0"] = stamp
        elapsed = stamp - state["t0"]
        state["last"] = stamp
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        odom_x, odom_y, odom_yaw = drift.update(position.x, position.y, yaw)
        if drop_after >= 0.0 and drop_after <= elapsed < drop_after + drop_duration:
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = odom_frame
        out.child_frame_id = base_frame
        out.pose.pose.position.x = odom_x
        out.pose.pose.position.y = odom_y
        out.pose.pose.orientation.z = math.sin(odom_yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(odom_yaw / 2.0)
        publisher.publish(out)

    node.create_subscription(Odometry, gt_topic, on_ground_truth, 50)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
