#!/usr/bin/env python3
"""仿真闭环评估：对比 /localization/state 与 /sim/ground_truth，输出 ATE/RPE。

指标：
  ate_rmse / ate_max   map 系平移误差（m）
  rpe_rmse             固定帧间隔相对平移误差（m）
  ok_ratio             state==OK 的采样占比
  transitions          状态迁移序列（含时间戳与原因）

超标时进程以 1 退出，可直接用于 CI 断言。
"""

import argparse
import csv
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sim_common import ate_rmse, rpe_rmse

STATE_NAMES = {0: "INITIALIZING", 1: "OK", 2: "DEGRADED", 3: "LOST"}


class Collector:
    def __init__(self):
        self.samples = []
        self.ground_truth = []

    def add_state(self, stamp, x, y, state, confidence, reason):
        self.samples.append(
            {
                "t": stamp,
                "x": x,
                "y": y,
                "state": int(state),
                "confidence": float(confidence),
                "reason": reason,
            }
        )

    def add_truth(self, stamp, x, y):
        self.ground_truth.append({"t": stamp, "x": x, "y": y})

    def summary(self, rpe_step=5):
        gt = self.ground_truth
        matched = []
        for sample in self.samples:
            best = None
            for item in gt:
                delta = abs(item["t"] - sample["t"])
                if best is None or delta < best[0]:
                    best = (delta, item)
            if best is not None and best[0] <= 0.15:
                matched.append((sample, best[1]))

        estimated = [(s["x"], s["y"]) for s, _ in matched]
        truth = [(g["x"], g["y"]) for _, g in matched]
        ate, ate_max, count = ate_rmse(estimated, truth)
        rpe = rpe_rmse(estimated, truth, rpe_step) if count > rpe_step else None

        transitions = []
        previous = None
        for sample in self.samples:
            if sample["state"] != previous:
                transitions.append(
                    {
                        "t": round(sample["t"], 3),
                        "state": STATE_NAMES.get(sample["state"], str(sample["state"])),
                        "reason": sample["reason"],
                    }
                )
                previous = sample["state"]

        ok = sum(1 for s in self.samples if s["state"] == 1)
        lost_ok_latency = None
        lost_at = None
        for sample in self.samples:
            if sample["state"] == 3:
                if lost_at is None:
                    lost_at = sample["t"]
            elif sample["state"] == 1 and lost_at is not None:
                lost_ok_latency = sample["t"] - lost_at
                lost_at = None

        return {
            "samples": len(self.samples),
            "matched": count,
            "ate_rmse": None if count == 0 else round(ate, 4),
            "ate_max": None if count == 0 else round(ate_max, 4),
            "rpe_rmse": None if rpe is None else round(rpe, 4),
            "ok_ratio": round(ok / len(self.samples), 4) if self.samples else 0.0,
            "transitions": transitions,
            "lost_to_ok_sec": (
                None if lost_ok_latency is None else round(lost_ok_latency, 3)
            ),
        }


def load_csv(collector, samples_path, truth_path):
    """离线模式：从 CSV 回放已录制的采样，不依赖 ROS。"""
    with open(samples_path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            collector.add_state(
                float(row["t"]),
                float(row["x"]),
                float(row["y"]),
                int(row.get("state", 1)),
                float(row.get("confidence", 1.0)),
                row.get("reason", ""),
            )
    with open(truth_path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            collector.add_truth(float(row["t"]), float(row["x"]), float(row["y"]))


def run(args, collector):
    import rclpy
    from nav_msgs.msg import Odometry

    from foray_localization_ros.msg import LocalizationState

    rclpy.init()
    node = rclpy.create_node("eval_trajectory")

    def on_state(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        collector.add_state(
            stamp, position.x, position.y, msg.state, msg.confidence, msg.reason
        )

    def on_truth(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        collector.add_truth(stamp, position.x, position.y)

    node.create_subscription(LocalizationState, "/localization/state", on_state, 50)
    node.create_subscription(Odometry, "/sim/ground_truth", on_truth, 50)

    if args.duration > 0.0:
        deadline = node.get_clock().now().nanoseconds * 1e-9 + args.duration
        timer = node.create_timer(
            0.2,
            lambda: (
                rclpy.shutdown()
                if node.get_clock().now().nanoseconds * 1e-9 >= deadline
                else None
            ),
        )
    else:
        timer = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if timer is not None:
            node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=float, default=30.0, help="采集时长 s（0=手动结束）"
    )
    parser.add_argument("--max-ate", type=float, default=0.15, help="ATE RMSE 上限 m")
    parser.add_argument("--min-ok-ratio", type=float, default=0.90, help="OK 占比下限")
    parser.add_argument("--min-samples", type=int, default=20, help="最少有效采样")
    parser.add_argument("--rpe-step", type=int, default=5, help="RPE 帧间隔")
    parser.add_argument("--json", help="结果写入 JSON 路径")
    parser.add_argument(
        "--offline",
        action="store_true",
        help="不启动 ROS：从 --samples/--truth CSV 读取",
    )
    parser.add_argument(
        "--samples", help="离线采样 CSV（t,x,y,state,confidence,reason）"
    )
    parser.add_argument("--truth", help="离线真值 CSV（t,x,y）")
    args = parser.parse_args(argv)

    collector = Collector()
    if args.offline:
        if not args.samples or not args.truth:
            parser.error("--offline 需要 --samples 与 --truth")
        load_csv(collector, args.samples, args.truth)
    else:
        run(args, collector)
    summary = collector.summary(args.rpe_step)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.json:
        pathlib.Path(args.json).write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    failures = []
    if summary["matched"] < args.min_samples:
        failures.append(f"有效采样不足: {summary['matched']} < {args.min_samples}")
    if summary["ate_rmse"] is None or summary["ate_rmse"] > args.max_ate:
        failures.append(f"ATE 超标: {summary['ate_rmse']} > {args.max_ate}")
    if summary["ok_ratio"] < args.min_ok_ratio:
        failures.append(f"OK 占比不足: {summary['ok_ratio']} < {args.min_ok_ratio}")
    if failures:
        for item in failures:
            print(f"[FAIL] {item}", file=sys.stderr)
        return 1
    print("[PASS] 仿真闭环指标达标")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
