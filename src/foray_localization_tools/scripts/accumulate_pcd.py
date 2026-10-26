#!/usr/bin/env python3
"""按真值位姿累积多帧点云，生成先验 PCD 地图。

配合 sim_replay.py --dump-dir 的离线产物使用：scans/*.pcd + poses.csv。
Gazebo 模式也可用真值位姿 + /livox/lidar 录制的帧离线累积。
"""

import argparse
import csv
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sim_common import read_pcd, write_pcd


def read_poses(path):
    poses = {}
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            stamp = float(row["timestamp"])
            poses[stamp] = (
                float(row["x"]),
                float(row["y"]),
                float(row["z"]),
                float(row["yaw"]),
            )
    return poses


def accumulate(scans, poses, voxel):
    merged = {}
    for scan_path in scans:
        stamp = float(scan_path.stem.split("_")[-1])
        pose = poses.get(stamp)
        if pose is None:
            continue
        x, y, z, yaw = pose
        cy, sy = math.cos(yaw), math.sin(yaw)
        for px, py, pz in read_pcd(scan_path):
            wx = x + cy * px - sy * py
            wy = y + sy * px + cy * py
            wz = z + pz
            key = (
                math.floor(wx / voxel),
                math.floor(wy / voxel),
                math.floor(wz / voxel),
            )
            if key not in merged:
                cell = (key[0] * voxel, key[1] * voxel, key[2] * voxel)
                merged[key] = cell
    return list(merged.values())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scans", required=True, help="点云目录（*_<stamp>.pcd）")
    parser.add_argument("--poses", required=True, help="真值位姿 CSV")
    parser.add_argument("--output", required=True, help="输出 PCD")
    parser.add_argument("--voxel", type=float, default=0.05, help="体素边长 m")
    args = parser.parse_args(argv)

    if args.voxel <= 0.0:
        parser.error("voxel 必须为正")
    scan_dir = pathlib.Path(args.scans)
    scans = sorted(scan_dir.glob("*.pcd"))
    if not scans:
        print(f"目录中没有 PCD: {scan_dir}", file=sys.stderr)
        return 1
    poses = read_poses(args.poses)
    points = accumulate(scans, poses, args.voxel)
    if not points:
        print("没有可累积的点（检查时间戳是否匹配）", file=sys.stderr)
        return 1
    write_pcd(args.output, points)
    print(f"累积 {len(scans)} 帧，输出 {len(points)} 点 → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
