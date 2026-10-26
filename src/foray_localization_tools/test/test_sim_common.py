"""合成仿真工具链的几何与指标单测（不依赖 ROS）。"""

import math

import pytest
import sim_common as sim


def test_wrap_angle_boundaries():
    assert sim.wrap_angle(0.0) == pytest.approx(0.0)
    assert sim.wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert sim.wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_transform_inverse_roundtrip():
    pose = sim.make_transform(1.0, -2.0, 0.3, 0.1, -0.2, 0.7)
    identity = sim.transform_pose(pose, sim.invert_transform(pose))
    for row in range(4):
        for col in range(4):
            expected = 1.0 if row == col else 0.0
            assert identity[row][col] == pytest.approx(expected, abs=1e-9)


def test_pose_roundtrip_yaw():
    pose = sim.pose_from_xy_yaw(3.0, -1.0, 1.1)
    x, y, z, roll, pitch, yaw = sim.to_xyz_rpy(pose)
    assert (x, y, z) == pytest.approx((3.0, -1.0, 0.0))
    assert (roll, pitch) == pytest.approx((0.0, 0.0))
    assert yaw == pytest.approx(1.1)


def test_trajectory_starts_at_origin_heading_zero():
    x, y, _, _, _, yaw = sim.to_xyz_rpy(sim.Trajectory().pose_at(0.0))
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert yaw == pytest.approx(0.0)


def test_pcd_roundtrip(tmp_path):
    points = [(0.1, 0.2, 0.3), (-1.0, 2.0, -0.5)]
    path = tmp_path / "cloud.pcd"
    sim.write_pcd(path, points)
    loaded = sim.read_pcd(path)
    assert len(loaded) == 2
    assert loaded[0] == pytest.approx(points[0], abs=1e-6)


def test_range_image_and_raycast():
    # 半径 5 m 的圆柱面，传感器在原点：所有方向距离都应接近 5 m
    points = []
    for i in range(720):
        angle = 2.0 * math.pi * i / 720.0
        for k in range(21):
            z = -2.0 + 0.2 * k
            points.append((5.0 * math.cos(angle), 5.0 * math.sin(angle), z))
    image = sim.RangeImage(points)
    rng = sim.random.Random(0)
    cloud = sim.raycast(
        image,
        sim.pose_from_xy_yaw(0.0, 0.0, 0.0),
        300,
        rng,
        noise_sigma=0.0,
        dropout=0.0,
    )
    assert len(cloud) > 80
    for x, y, _ in cloud[:50]:
        assert math.hypot(x, y) == pytest.approx(5.0, abs=0.5)


def test_drift_model_zero_error_is_identity():
    drift = sim.DriftModel(
        scale_error=0.0, yaw_drift_deg_per_m=0.0, yaw_noise_deg=0.0, pos_noise=0.0
    )
    true_pose = sim.pose_from_xy_yaw(0.0, 0.0, 0.0)
    drift.update(true_pose)
    for i in range(1, 5):
        true_pose = sim.pose_from_xy_yaw(0.1 * i, 0.0, 0.0)
        odom = drift.update(true_pose)
        x, y, _, _, _, yaw = sim.to_xyz_rpy(odom)
        assert x == pytest.approx(0.1 * i)
        assert y == pytest.approx(0.0)
        assert yaw == pytest.approx(0.0)


def test_drift_model_scale_error_accumulates():
    drift = sim.DriftModel(
        scale_error=0.1, yaw_drift_deg_per_m=0.0, yaw_noise_deg=0.0, pos_noise=0.0
    )
    drift.update(sim.pose_from_xy_yaw(0.0, 0.0, 0.0))
    for i in range(1, 11):
        odom = drift.update(sim.pose_from_xy_yaw(0.1 * i, 0.0, 0.0))
    x, _, _, _, _, _ = sim.to_xyz_rpy(odom)
    assert x == pytest.approx(1.1, rel=1e-6)


def test_ate_and_rpe_metrics():
    truth = [(float(i), 0.0) for i in range(12)]
    estimated = [(float(i), 0.0) for i in range(11)] + [(11.0, 0.1)]
    rmse, worst, count = sim.ate_rmse(estimated, truth)
    assert count == 12
    assert rmse == pytest.approx(math.sqrt(0.01 / 12))
    assert worst == pytest.approx(0.1)
    assert sim.rpe_rmse(truth, truth, step=5) == pytest.approx(0.0)
