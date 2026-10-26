#!/usr/bin/env python3
"""生成 12 m × 8 m 矩形场地 + 箱体的合成 PCD，供无 Gazebo 的离线闭环使用。"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from sim_common import make_demo_field


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="输出 PCD 路径")
    parser.add_argument("--width", type=float, default=12.0, help="场地宽度 m")
    parser.add_argument("--height", type=float, default=8.0, help="场地深度 m")
    parser.add_argument("--spacing", type=float, default=0.12, help="点间距 m")
    args = parser.parse_args(argv)

    if args.width <= 0.0 or args.height <= 0.0:
        parser.error("尺寸必须为正")
    pathlib.Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    count = make_demo_field(args.output, args.width, args.height, spacing=args.spacing)
    print(f"已写入 {args.output}（{count} 点）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
