"""评估与指标路径单测：不启动 ROS，直接跑离线 CSV 模式。"""

import csv
import math

import pytest

import eval_trajectory as ev


def build_collector(drift=0.0, ok_states=(1,) * 10):
    collector = ev.Collector()
    for i, state in enumerate(ok_states):
        t = 0.1 * i
        collector.add_state(t, 0.1 * i, drift, state, 1.0, "" if state == 1 else "test")
        collector.add_truth(t, 0.1 * i, 0.0)
    return collector


def test_summary_perfect_tracking():
    summary = build_collector().summary()
    assert summary["samples"] == 10
    assert summary["matched"] == 10
    assert summary["ate_rmse"] == pytest.approx(0.0, abs=1e-6)
    assert summary["ok_ratio"] == pytest.approx(1.0)


def test_summary_ate_reflects_drift():
    summary = build_collector(drift=0.2).summary()
    assert summary["ate_rmse"] == pytest.approx(0.2, abs=1e-6)
    assert summary["ate_max"] == pytest.approx(0.2, abs=1e-6)


def test_transitions_and_lost_to_ok_latency():
    states = (1, 1, 2, 3, 3, 3, 1, 1)
    collector = ev.Collector()
    for i, state in enumerate(states):
        collector.add_state(0.1 * i, 0.0, 0.0, state, 0.5, "r%d" % state)
        collector.add_truth(0.1 * i, 0.0, 0.0)
    summary = collector.summary()
    names = [item["state"] for item in summary["transitions"]]
    assert names == ["OK", "DEGRADED", "LOST", "OK"]
    # LOST 出现在 t=0.3，恢复 OK 在 t=0.6
    assert summary["lost_to_ok_sec"] == pytest.approx(0.3, abs=1e-6)


def test_offline_cli_pass_and_fail(tmp_path):
    samples = tmp_path / "samples.csv"
    truth = tmp_path / "truth.csv"
    with samples.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "state", "confidence", "reason"])
        for i in range(30):
            writer.writerow([0.1 * i, 0.05 * i, 0.0, 1, 1.0, ""])
    with truth.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y"])
        for i in range(30):
            writer.writerow([0.1 * i, 0.05 * i, 0.0])

    assert ev.main(["--offline", "--samples", str(samples), "--truth", str(truth)]) == 0

    with truth.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y"])
        for i in range(30):
            writer.writerow([0.1 * i, 0.05 * i + 1.0, 0.0])
    assert ev.main(["--offline", "--samples", str(samples), "--truth", str(truth)]) == 1
