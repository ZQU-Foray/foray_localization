#!/usr/bin/env python3
"""无 Gazebo/无 GPU 的合成数据回放：从先验 PCD 射线投射生成点云与里程计。

发布（--ros）：
  /livox/lidar           sensor_msgs/PointCloud2   livox_frame
  /aft_mapped_to_init    nav_msgs/Odometry         odom -> base_footprint
  /sim/ground_truth      nav_msgs/Odometry         map  -> base_footprint
  /tf_static             base_footprint -> livox_frame

离线（--dump-dir）：写出 scans/<stamp>.pcd、poses.csv、odom.csv，
供 accumulate_pcd.py 建图与无 ROS 的回归测试使用。
"""

import argparse
import csv
import pathlib
import random
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sim_common import (
    DriftModel,
    RangeImage,
    Trajectory,
    make_transform,
    quaternion_from_yaw,
    raycast,
    read_manifest,
    read_pcd,
    to_xyz_rpy,
    transform_pose,
    write_pcd,
)

LIDAR_OFFSET_Z = 0.20  # MID360 安装高度（m），与 bringup 的 URDF 一致
BASE_TO_LIDAR = make_transform(0.0, 0.0, LIDAR_OFFSET_Z, 0.0, 0.0, 0.0)


def build_world(manifest_path):
    manifest = read_manifest(manifest_path)
    pcd_path = pathlib.Path(manifest_path).parent / manifest["pcd_file"]
    points = read_pcd(pcd_path)
    return manifest, points


def dump_offline(args, world, trajectory):
    rng = random.Random(args.seed)
    drift = DriftModel(seed=args.seed)
    out = pathlib.Path(args.dump_dir)
    (out / "scans").mkdir(parents=True, exist_ok=True)
    range_image = RangeImage(world)
    ticks = int(args.duration * args.odom_rate)
    with open(out / "poses.csv", "w", newline="", encoding="utf-8") as poses, open(
        out / "odom.csv", "w", newline="", encoding="utf-8"
    ) as odoms:
        pose_writer = csv.writer(poses)
        odom_writer = csv.writer(odoms)
        pose_writer.writerow(["timestamp", "x", "y", "z", "yaw"])
        odom_writer.writerow(["timestamp", "x", "y", "z", "yaw"])
        next_cloud = 0.0
        for tick in range(ticks + 1):
            t = tick / args.odom_rate
            true_pose = trajectory.pose_at(t)
            dropped = is_dropped(args, t)
            if not dropped:
                odom = drift.update(true_pose)
                x, y, z, _, _, yaw = to_xyz_rpy(odom)
                odom_writer.writerow(
                    [f"{t:.6f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{yaw:.6f}"]
                )
            x, y, z, _, _, yaw = to_xyz_rpy(true_pose)
            pose_writer.writerow(
                [f"{t:.6f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{yaw:.6f}"]
            )
            if t + 1e-9 >= next_cloud:
                next_cloud += 1.0 / args.cloud_rate
                lidar_pose = transform_pose(true_pose, BASE_TO_LIDAR)
                cloud = raycast(
                    range_image, lidar_pose, args.points, rng, noise_sigma=args.noise
                )
                write_pcd(out / "scans" / f"cloud_{t:.6f}.pcd", cloud)
    print(f"离线数据已写入 {out}（{ticks + 1} 个位姿采样）")
    return 0


def is_dropped(args, t):
    if args.drop_odom_after < 0.0:
        return False
    return args.drop_odom_after <= t < args.drop_odom_after + args.drop_odom_duration


def run_ros(args, world, trajectory):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("sim_replay")
    cloud_pub = node.create_publisher(PointCloud2, "/livox/lidar", 10)
    odom_pub = node.create_publisher(Odometry, "/aft_mapped_to_init", 10)
    truth_pub = node.create_publisher(Odometry, "/sim/ground_truth", 10)
    static_tf = StaticTransformBroadcaster(node)

    stamp = node.get_clock().now().to_msg()
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "livox_frame"
    transform.transform.translation.z = LIDAR_OFFSET_Z
    transform.transform.rotation.w = 1.0
    static_tf.sendTransform(transform)

    rng = random.Random(args.seed)
    drift = DriftModel(seed=args.seed)
    range_image = RangeImage(world)
    state = {"tick": 0, "next_cloud": 0.0, "start": time.monotonic()}

    def make_odometry(header_frame, child, pose, stamp_msg):
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = header_frame
        odom.child_frame_id = child
        x, y, z, _, _, yaw = to_xyz_rpy(pose)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        return odom

    def on_tick():
        t = state["tick"] / args.odom_rate
        if t > args.duration:
            node.get_logger().info("回放结束")
            rclpy.shutdown()
            return
        true_pose = trajectory.pose_at(t)
        stamp_msg = node.get_clock().now().to_msg()
        truth_pub.publish(make_odometry("map", "base_footprint", true_pose, stamp_msg))
        odom = drift.update(true_pose)
        if not is_dropped(args, t):
            odom_pub.publish(make_odometry("odom", "base_footprint", odom, stamp_msg))
        if t + 1e-9 >= state["next_cloud"]:
            state["next_cloud"] += 1.0 / args.cloud_rate
            lidar_pose = transform_pose(true_pose, BASE_TO_LIDAR)
            cloud = raycast(
                range_image, lidar_pose, args.points, rng, noise_sigma=args.noise
            )
            header = Header()
            header.stamp = stamp_msg
            header.frame_id = "livox_frame"
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, cloud))
        state["tick"] += 1
        # 保持实时：按名义时间推进，落后时不补帧
        expected = state["start"] + t
        delay = expected - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)

    timer = node.create_timer(1.0 / args.odom_rate, on_tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, help="先验地图 manifest.yaml")
    parser.add_argument("--duration", type=float, default=60.0, help="回放时长 s")
    parser.add_argument("--cloud-rate", type=float, default=10.0, help="点云频率 Hz")
    parser.add_argument("--odom-rate", type=float, default=50.0, help="里程计频率 Hz")
    parser.add_argument("--points", type=int, default=3000, help="每帧射线数")
    parser.add_argument("--noise", type=float, default=0.02, help="测距噪声 sigma m")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    parser.add_argument(
        "--drop-odom-after",
        type=float,
        default=-1.0,
        help="在 t 秒后人为中断里程计（-1 关闭）",
    )
    parser.add_argument(
        "--drop-odom-duration", type=float, default=1.0, help="中断持续时间 s"
    )
    parser.add_argument("--ros", action="store_true", help="发布 ROS 2 话题")
    parser.add_argument("--dump-dir", help="离线写出 scans/poses.csv/odom.csv")
    args = parser.parse_args(argv)

    if args.cloud_rate <= 0.0 or args.odom_rate <= 0.0:
        parser.error("频率必须为正")
    if args.points <= 0:
        parser.error("points 必须为正")

    _, points = build_world(args.manifest)
    trajectory = Trajectory()
    if args.dump_dir:
        return dump_offline(args, points, trajectory)
    if args.ros:
        return run_ros(args, points, trajectory)
    parser.error("需要 --ros 或 --dump-dir 之一")


if __name__ == "__main__":
    raise SystemExit(main())
