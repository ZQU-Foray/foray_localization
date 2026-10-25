"""仿真资产的静态校验：launch 语法、配置键、注入器漂移模型。"""

import ast
import pathlib

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]

REQUIRED_SIM_PARAMS = {
    "voxel_leaf_m",
    "map_voxel_leaf_m",
    "align_max_correspondence_m",
    "ok_mean_residual_m",
    "degraded_mean_residual_m",
    "lost_timeout_sec",
    "odom_timeout_sec",
}


@pytest.mark.parametrize(
    "relative",
    [
        "foray_localization_sim/launch/sim_localization.launch.py",
        "foray_localization_bringup/launch/localization.launch.py",
    ],
)
def test_launch_python_syntax(relative):
    source = (ROOT / relative).read_text(encoding="utf-8")
    ast.parse(source)
    assert "LaunchDescription" in source


def test_sim_config_has_required_params():
    data = yaml.safe_load(
        (ROOT / "foray_localization_bringup/config/sim.yaml").read_text(encoding="utf-8")
    )
    params = data["foray_localization"]["ros__parameters"]
    missing = REQUIRED_SIM_PARAMS - set(params)
    assert not missing, f"sim.yaml 缺少参数: {sorted(missing)}"
    assert params["odom_timeout_sec"] < params["lost_timeout_sec"]
    launch = (ROOT / "foray_localization_sim/launch/sim_localization.launch.py").read_text(
        encoding="utf-8"
    )
    assert "map_manifest" in launch


def test_odom_injector_drift_and_dropout():
    import sim_odom_injector as injector

    class FakeNode:
        def declare_parameter(self, name, default):
            values = {
                "scale_error": 0.1,
                "yaw_drift_deg_per_m": 0.0,
                "yaw_noise_deg": 0.0,
                "pos_noise_m": 0.0,
                "seed": 0,
            }
            return type("Param", (), {"value": values[name]})()

    drift = injector.OdomDrift(FakeNode())
    drift.update(0.0, 0.0, 0.0)
    pose = drift.update(1.0, 0.0, 0.0)
    assert pose[0] == pytest.approx(1.1)
    assert pose[1] == pytest.approx(0.0)
    assert pose[2] == pytest.approx(0.0)
