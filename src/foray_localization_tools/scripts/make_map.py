#!/usr/bin/env python3
"""从 PCD 点云生成 2D 栅格与 manifest.yaml（D4：2D 与 3D 同源）。

坐标系约定：地图原点 = 场地几何中心；origin 字段是栅格 (0,0) 单元的世界坐标。
栅格单元以 origin 为左下角，+X 向右、+Y 向上；PGM 行 0 为顶部，写出时翻转 Y。
"""

import argparse
import datetime
import hashlib
import math
import pathlib
import shutil
import struct
import sys

import yaml

OCCUPIED = 0
UNKNOWN = 128


def read_pcd(path):
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
        for line in data[offset:].decode("ascii", "replace").splitlines():
            values = line.split()
            if len(values) < len(fields):
                continue
            record = dict(zip(fields, values))
            xyz.append((float(record["x"]), float(record["y"]), float(record["z"])))
    elif fmt == "binary":
        stride = sum(s * c for s, c in zip(sizes, counts))
        offsets = []
        cursor = 0
        for size, count in zip(sizes, counts):
            offsets.append((cursor, size, count))
            cursor += size * count
        index = {name: i for i, name in enumerate(fields)}

        def component(record, spec):
            start, size, _ = spec
            if size not in (4, 8):
                raise ValueError(f"PCD 字段宽度不支持: {size}")
            code = "f" if size == 4 else "d"
            return struct.unpack_from("<" + code, record, start)[0]

        for name in ("x", "y", "z"):
            if types[index[name]] != "F":
                raise ValueError(f"PCD 字段 {name} 不是浮点类型")

        x_off = offsets[index["x"]]
        y_off = offsets[index["y"]]
        z_off = offsets[index["z"]]
        for i in range(points):
            record = data[offset + i * stride : offset + (i + 1) * stride]
            if len(record) < stride:
                break
            xyz.append(
                (
                    component(record, x_off),
                    component(record, y_off),
                    component(record, z_off),
                )
            )
    else:
        raise ValueError(f"不支持的 PCD DATA 类型: {fmt}")

    if not xyz:
        raise ValueError("PCD 中没有点")
    return xyz


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_grid(points, resolution, field_size, center, z_min, z_max):
    width = math.ceil(field_size[0] / resolution)
    height = math.ceil(field_size[1] / resolution)
    origin = (
        center[0] - field_size[0] / 2.0,
        center[1] - field_size[1] / 2.0,
        0.0,
    )
    cells = bytearray([UNKNOWN] * (width * height))
    for x, y, z in points:
        if z < z_min or z > z_max:
            continue
        ix = math.floor((x - origin[0]) / resolution)
        iy = math.floor((y - origin[1]) / resolution)
        if 0 <= ix < width and 0 <= iy < height:
            cells[iy * width + ix] = OCCUPIED
    return width, height, origin, cells


def write_pgm(path, width, height, cells):
    path = pathlib.Path(path)
    with open(path, "wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        handle.writelines(
            bytes(cells[row * width : (row + 1) * width])
            for row in range(height - 1, -1, -1)
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True, help="输入 PCD（ascii 或 binary）")
    parser.add_argument("--output-dir", required=True, help="输出目录")
    parser.add_argument("--version", default="rmul-2027-3v3", help="地图版本号")
    parser.add_argument(
        "--resolution", type=float, default=0.05, help="栅格分辨率 m/cell"
    )
    parser.add_argument(
        "--field-size", type=float, nargs=2, default=(12.0, 8.0), help="场地尺寸 m m"
    )
    parser.add_argument(
        "--center", type=float, nargs=2, default=(0.0, 0.0), help="场地中心世界坐标"
    )
    parser.add_argument("--z-min", type=float, default=0.10, help="投影高度下限 m")
    parser.add_argument("--z-max", type=float, default=2.00, help="投影高度上限 m")
    parser.add_argument("--source", default="manual", help="地图来源说明")
    args = parser.parse_args(argv)

    if args.resolution <= 0.0 or args.resolution > 1.0:
        parser.error("resolution 必须在 (0, 1.0] 内")
    if args.z_max <= args.z_min:
        parser.error("z-max 必须大于 z-min")

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    pcd_path = pathlib.Path(args.pcd).resolve()
    points = read_pcd(pcd_path)
    width, height, origin, cells = build_grid(
        points,
        args.resolution,
        args.field_size,
        args.center,
        args.z_min,
        args.z_max,
    )

    # manifest 只允许同目录相对路径：把 PCD 复制进地图目录后在原地算哈希。
    copied_pcd = output_dir / pcd_path.name
    if copied_pcd.resolve() != pcd_path:
        shutil.copy2(pcd_path, copied_pcd)

    grid_name = "map.pgm"
    grid_path = output_dir / grid_name
    write_pgm(grid_path, width, height, cells)

    manifest = {
        "version": args.version,
        "frame_id": "map",
        "origin": [float(v) for v in origin],
        "resolution": float(args.resolution),
        "width": width,
        "height": height,
        "pcd_file": copied_pcd.name,
        "grid_file": grid_name,
        "pcd_sha256": sha256_file(copied_pcd),
        "grid_sha256": sha256_file(grid_path),
        "created_at": "1970-01-01T00:00:00Z",
        "source": args.source,
    }
    manifest["created_at"] = (
        datetime.datetime.now(datetime.UTC).replace(microsecond=0).isoformat()
    )
    manifest_path = output_dir / "manifest.yaml"
    with open(manifest_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(manifest, handle, allow_unicode=True, sort_keys=False)

    occupied = sum(1 for cell in cells if cell == OCCUPIED)
    print(
        f"地图已生成: {manifest_path} ({width}x{height}, {occupied} 个占据单元, "
        f"{len(points)} 个点)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
