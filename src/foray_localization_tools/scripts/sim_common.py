#!/usr/bin/env python3
"""仿真验证共用的纯 Python 几何、点云与轨迹工具。

不依赖 ROS、numpy 或 PCL，便于在 CI 与离线环境复用。坐标约定与
foray_localization_core 一致：map 系原点为场地中心，+X 红方→蓝方，
+Y 红方左侧，+Z 向上；欧拉角顺序 Rz * Ry * Rx。
"""

import math
import pathlib
import random
import struct

import yaml

AZ_BINS = 720  # 0.5 度：兼顾合成点云密度
EL_MIN_DEG = -10.0
EL_MAX_DEG = 60.0
EL_BINS = 36
MAX_RANGE_M = 40.0


def wrap_angle(angle):
    """归一化到 (-pi, pi]。"""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def make_transform(x, y, z, roll, pitch, yaw):
    rot = rotation_matrix(roll, pitch, yaw)
    return [
        [rot[0][0], rot[0][1], rot[0][2], x],
        [rot[1][0], rot[1][1], rot[1][2], y],
        [rot[2][0], rot[2][1], rot[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def pose_from_xy_yaw(x, y, yaw, z=0.0):
    return make_transform(x, y, z, 0.0, 0.0, yaw)


def transform_pose(parent, child):
    """parent * child。"""
    out = [[0.0] * 4 for _ in range(4)]
    for row in range(3):
        for col in range(4):
            out[row][col] = sum(parent[row][k] * child[k][col] for k in range(4))
    out[3][3] = 1.0
    return out


def invert_transform(transform):
    rot = [[transform[j][i] for j in range(3)] for i in range(3)]
    trans = [transform[0][3], transform[1][3], transform[2][3]]
    out = [[0.0] * 4 for _ in range(4)]
    for row in range(3):
        for col in range(3):
            out[row][col] = rot[row][col]
        out[row][3] = -sum(rot[row][k] * trans[k] for k in range(3))
    out[3][3] = 1.0
    return out


def to_xyz_rpy(transform):
    x, y, z = transform[0][3], transform[1][3], transform[2][3]
    sin_pitch = max(-1.0, min(1.0, -transform[2][0]))
    pitch = math.asin(sin_pitch)
    if abs(sin_pitch) > 0.999999:
        roll = 0.0
        yaw = math.atan2(-transform[0][1], transform[1][1])
    else:
        roll = math.atan2(transform[2][1], transform[2][2])
        yaw = math.atan2(transform[1][0], transform[0][0])
    return x, y, z, roll, pitch, yaw


def yaw_of(transform):
    return math.atan2(transform[1][0], transform[0][0])


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def read_pcd(path):
    """读取 ascii/binary PCD 的 xyz，返回 [(x, y, z)]。"""
    data = pathlib.Path(path).read_bytes()
    header = {}
    offset = 0
    while offset < len(data):
        end = data.find(b"\n", offset)
        if end < 0:
            raise ValueError("PCD 头不完整")
        line = data[offset:end].decode("ascii", "replace").strip()
        offset = end + 1
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
        header[key.upper()] = value.strip()
        if key.upper() == "DATA":
            break

    fields = header.get("FIELDS", "x y z").split()
    sizes = [int(v) for v in header.get("SIZE", "4 4 4").split()]
    types = header.get("TYPE", "F F F").split()
    counts = [int(v) for v in header.get("COUNT", "1 1 1").split()]
    points = int(header.get("POINTS", header.get("WIDTH", "0")))
    fmt = header.get("DATA", "ascii").lower()
    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise ValueError("PCD 缺少 x/y/z 字段")

    xyz = []
    if fmt == "ascii":
        index = {name: i for i, name in enumerate(fields)}
        for line in data[offset:].decode("ascii", "replace").splitlines():
            values = line.split()
            if len(values) < len(fields):
                continue
            xyz.append(
                (
                    float(values[index["x"]]),
                    float(values[index["y"]]),
                    float(values[index["z"]]),
                )
            )
    elif fmt == "binary":
        stride = sum(s * c for s, c in zip(sizes, counts))
        offsets = []
        cursor = 0
        for size, count in zip(sizes, counts):
            offsets.append((cursor, size))
            cursor += size * count
        index = {name: i for i, name in enumerate(fields)}
        for name in ("x", "y", "z"):
            if types[index[name]] != "F":
                raise ValueError(f"PCD 字段 {name} 不是浮点类型")
        for i in range(points):
            record = data[offset + i * stride : offset + (i + 1) * stride]
            if len(record) < stride:
                break
            coords = []
            for name in ("x", "y", "z"):
                start, size = offsets[index[name]]
                code = "f" if size == 4 else "d"
                coords.append(struct.unpack_from("<" + code, record, start)[0])
            xyz.append(tuple(coords))
    else:
        raise ValueError(f"不支持的 PCD DATA 类型: {fmt}")

    if not xyz:
        raise ValueError("PCD 中没有点")
    return xyz


def write_pcd(path, points):
    """写 binary PCD（x y z float32）。"""
    path = pathlib.Path(path)
    body = bytearray()
    for x, y, z in points:
        body += struct.pack("<fff", x, y, z)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    path.write_bytes(header.encode("ascii") + bytes(body))


def read_manifest(path):
    return yaml.safe_load(pathlib.Path(path).read_text(encoding="utf-8"))


class RangeImage:
    """把先验点云压成方向-距离图，用于快速射线投射。"""

    def __init__(self, points, az_bins=AZ_BINS, el_bins=EL_BINS):
        self.az_bins = az_bins
        self.el_bins = el_bins
        self.ranges = [math.inf] * (az_bins * el_bins)
        for x, y, z in points:
            dist = math.sqrt(x * x + y * y + z * z)
            if dist <= 1e-6 or dist > MAX_RANGE_M:
                continue
            az = math.atan2(y, x)
            el = math.asin(max(-1.0, min(1.0, z / dist)))
            ia = int((az + math.pi) / (2.0 * math.pi) * az_bins) % az_bins
            ie = int(
                (math.degrees(el) - EL_MIN_DEG) / (EL_MAX_DEG - EL_MIN_DEG) * el_bins
            )
            if ie < 0 or ie >= el_bins:
                continue
            index = ie * az_bins + ia
            self.ranges[index] = min(self.ranges[index], dist)

    def range_at(self, azimuth, elevation):
        ia = int((azimuth + math.pi) / (2.0 * math.pi) * self.az_bins) % self.az_bins
        ie = int(
            (math.degrees(elevation) - EL_MIN_DEG)
            / (EL_MAX_DEG - EL_MIN_DEG)
            * self.el_bins
        )
        if ie < 0 or ie >= self.el_bins:
            return math.inf
        return self.ranges[ie * self.az_bins + ia]


def sample_ray_directions(count, rng):
    """模拟 MID360 非重复扫描：方位角均匀、俯仰角偏向低仰角。"""
    directions = []
    for _ in range(count):
        azimuth = rng.uniform(-math.pi, math.pi)
        u = rng.random()
        elevation = math.radians(EL_MIN_DEG + (EL_MAX_DEG - EL_MIN_DEG) * (u**1.6))
        directions.append((azimuth, elevation))
    return directions


def raycast(range_image, pose, count, rng, noise_sigma=0.02, dropout=0.02):
    """在 pose（map 系 base）下生成 livox_frame 点云。"""
    rot = [[pose[j][i] for j in range(3)] for i in range(3)]
    trans = (pose[0][3], pose[1][3], pose[2][3])
    cloud = []
    for azimuth, elevation in sample_ray_directions(count, rng):
        distance = range_image.range_at(azimuth, elevation)
        if not math.isfinite(distance):
            continue
        distance += rng.gauss(0.0, noise_sigma)
        if distance <= 0.1:
            continue
        if rng.random() < dropout:
            continue
        ce, se = math.cos(elevation), math.sin(elevation)
        ca, sa = math.cos(azimuth), math.sin(azimuth)
        dx, dy, dz = ce * ca, ce * sa, se
        # 世界点 = R * 传感器点 + t；传感器点 = R^T * (世界点 - t)
        wx = trans[0] + distance * (rot[0][0] * dx + rot[0][1] * dy + rot[0][2] * dz)
        wy = trans[1] + distance * (rot[1][0] * dx + rot[1][1] * dy + rot[1][2] * dz)
        wz = trans[2] + distance * (rot[2][0] * dx + rot[2][1] * dy + rot[2][2] * dz)
        px, py, pz = wx - trans[0], wy - trans[1], wz - trans[2]
        cloud.append(
            (
                rot[0][0] * px + rot[1][0] * py + rot[2][0] * pz,
                rot[0][1] * px + rot[1][1] * py + rot[2][1] * pz,
                rot[0][2] * px + rot[1][2] * py + rot[2][2] * pz,
            )
        )
    return cloud


class Trajectory:
    """场内李萨如轨迹：t=0 时位于 (0, 0, 0)，保证默认初始位姿可用。"""

    def __init__(self, amplitude=(2.5, 1.8), omega=(0.35, 0.22), speed=0.6):
        self.amplitude = amplitude
        self.omega = omega
        self.speed = speed

    def pose_at(self, t):
        ax, ay = self.amplitude
        wx, wy = self.omega
        # t=0 时位置与航向都为 0，匹配默认 initial_pose
        x = ax * math.sin(wx * t)
        y = ay * (1.0 - math.cos(wy * t))
        dx = ax * wx * math.cos(wx * t)
        dy = ay * wy * math.sin(wy * t)
        return pose_from_xy_yaw(x, y, math.atan2(dy, dx))


class DriftModel:
    """把真值增量改写成带尺度/航向/噪声误差的里程计增量。"""

    def __init__(
        self,
        scale_error=0.02,
        yaw_drift_deg_per_m=0.8,
        yaw_noise_deg=0.05,
        pos_noise=0.005,
        seed=0,
    ):
        self.scale_error = scale_error
        self.yaw_drift = math.radians(yaw_drift_deg_per_m)
        self.yaw_noise = math.radians(yaw_noise_deg)
        self.pos_noise = pos_noise
        self.rng = random.Random(seed)
        self.odom = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.last_true = None

    def update(self, true_pose):
        if self.last_true is None:
            self.last_true = true_pose
            self.odom = [row[:] for row in true_pose]
            return self.odom
        delta = transform_pose(invert_transform(self.last_true), true_pose)
        x, y, z, roll, pitch, yaw = to_xyz_rpy(delta)
        distance = math.sqrt(x * x + y * y)
        scale = 1.0 + self.scale_error
        yaw = yaw + self.yaw_drift * distance + self.rng.gauss(0.0, self.yaw_noise)
        x = x * scale + self.rng.gauss(0.0, self.pos_noise)
        y = y * scale + self.rng.gauss(0.0, self.pos_noise)
        noisy = make_transform(x, y, z, roll, pitch, yaw)
        self.odom = transform_pose(self.odom, noisy)
        self.last_true = true_pose
        return self.odom


def ate_rmse(estimated, ground_truth):
    """estimated/ground_truth: [(x, y)]，返回 (rmse, max, count)。"""
    count = min(len(estimated), len(ground_truth))
    if count == 0:
        return float("inf"), float("inf"), 0
    squared = 0.0
    worst = 0.0
    for i in range(count):
        dx = estimated[i][0] - ground_truth[i][0]
        dy = estimated[i][1] - ground_truth[i][1]
        err = math.sqrt(dx * dx + dy * dy)
        squared += err * err
        worst = max(worst, err)
    return math.sqrt(squared / count), worst, count


def rpe_rmse(estimated, ground_truth, step=10):
    """固定帧间隔的相对位姿误差（平移，m）。"""
    pairs = []
    for i in range(0, min(len(estimated), len(ground_truth)) - step, step):
        j = i + step
        gt_dx = ground_truth[j][0] - ground_truth[i][0]
        gt_dy = ground_truth[j][1] - ground_truth[i][1]
        es_dx = estimated[j][0] - estimated[i][0]
        es_dy = estimated[j][1] - estimated[i][1]
        pairs.append(math.hypot(es_dx - gt_dx, es_dy - gt_dy))
    if not pairs:
        return float("inf")
    return math.sqrt(sum(p * p for p in pairs) / len(pairs))


def make_demo_field(path, width=12.0, height=8.0, wall_height=1.5, spacing=0.12):
    """生成矩形场地 + 若干箱体的点云，作为无 Gazebo 时的先验地图。"""
    points = []

    def line(x0, y0, x1, y1, z0, z1):
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(length / spacing))
        layers = max(1, int((z1 - z0) / spacing))
        for i in range(steps + 1):
            x = x0 + (x1 - x0) * i / steps
            y = y0 + (y1 - y0) * i / steps
            for k in range(layers + 1):
                points.append((x, y, z0 + (z1 - z0) * k / layers))

    hx, hy = width / 2.0, height / 2.0
    line(-hx, -hy, hx, -hy, 0.0, wall_height)
    line(-hx, hy, hx, hy, 0.0, wall_height)
    line(-hx, -hy, -hx, hy, 0.0, wall_height)
    line(hx, -hy, hx, hy, 0.0, wall_height)

    for cx, cy, sx, sy, sz in [(-3.0, 1.5, 0.8, 0.8, 0.7), (3.0, -1.5, 1.0, 0.6, 0.9)]:
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0
        line(x0, y0, x1, y0, 0.0, sz)
        line(x0, y1, x1, y1, 0.0, sz)
        line(x0, y0, x0, y1, 0.0, sz)
        line(x1, y0, x1, y1, 0.0, sz)

    write_pcd(path, points)
    return len(points)
