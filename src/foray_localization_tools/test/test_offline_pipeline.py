"""离线闭环端到端：合成场地 → 建图 → 回放 dump → 真值累积建图。

用 subprocess 真跑各脚本的 CLI（等同 ros2 run 入口），覆盖 argparse 与
install 布局下的模块导入；不依赖 ROS 运行时。
"""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "foray_localization_tools" / "scripts"


def run_script(name, *args):
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / name), *args],
        check=False,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, f"{name} 失败:\n{result.stdout}\n{result.stderr}"
    return result


def test_offline_pipeline_end_to_end(tmp_path):
    field = tmp_path / "field.pcd"
    run_script("gen_demo_field.py", "--output", str(field), "--spacing", "0.20")
    assert field.is_file() and field.stat().st_size > 0

    map_dir = tmp_path / "map"
    run_script(
        "make_map.py",
        "--pcd",
        str(field),
        "--output-dir",
        str(map_dir),
        "--version",
        "pytest-offline",
        "--source",
        "synthetic",
    )
    manifest = map_dir / "manifest.yaml"
    assert manifest.is_file()

    dump = tmp_path / "dump"
    run_script(
        "sim_replay.py",
        "--manifest",
        str(manifest),
        "--duration",
        "1.0",
        "--odom-rate",
        "20",
        "--cloud-rate",
        "5",
        "--points",
        "300",
        "--dump-dir",
        str(dump),
    )
    scans = sorted((dump / "scans").glob("*.pcd"))
    # duration=1.0 s、cloud_rate=5 Hz → t = 0, 0.2, ..., 1.0 共 6 帧
    assert len(scans) == 6
    assert (dump / "poses.csv").is_file()
    assert (dump / "odom.csv").is_file()

    accumulated = tmp_path / "accumulated.pcd"
    run_script(
        "accumulate_pcd.py",
        "--scans",
        str(dump / "scans"),
        "--poses",
        str(dump / "poses.csv"),
        "--output",
        str(accumulated),
    )
    assert accumulated.is_file() and accumulated.stat().st_size > 0

    final_dir = tmp_path / "map_final"
    run_script(
        "make_map.py",
        "--pcd",
        str(accumulated),
        "--output-dir",
        str(final_dir),
        "--version",
        "pytest-offline-acc",
        "--source",
        "synthetic",
    )
    validation = run_script("validate_map.py", str(final_dir / "manifest.yaml"))
    assert "地图校验通过" in validation.stdout


def test_eval_trajectory_rejects_missing_offline_inputs():
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / "eval_trajectory.py"), "--offline"],
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode != 0
    assert "--samples" in result.stderr
