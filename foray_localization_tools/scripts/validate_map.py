#!/usr/bin/env python3
"""校验先验地图 manifest 与文件一致性（供 CI 与上机前检查）。"""

import argparse
import hashlib
import pathlib
import sys

import yaml

REQUIRED = (
    "version",
    "frame_id",
    "origin",
    "resolution",
    "width",
    "height",
    "pcd_file",
    "grid_file",
    "pcd_sha256",
    "grid_sha256",
    "created_at",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_pgm_header(path):
    with open(path, "rb") as handle:
        magic = handle.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"不是 PGM: {path}")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        max_value = int(handle.readline())
    return width, height, max_value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", help="manifest.yaml 路径")
    parser.add_argument("--skip-hash", action="store_true", help="跳过 SHA-256 校验")
    args = parser.parse_args(argv)

    path = pathlib.Path(args.manifest).resolve()
    with open(path, encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle)

    errors = []
    for key in REQUIRED:
        if key not in manifest:
            errors.append(f"缺少字段 {key}")

    if manifest.get("frame_id") != "map":
        errors.append("frame_id 必须为 map")

    resolution = float(manifest.get("resolution", 0.0))
    if not 0.0 < resolution <= 1.0:
        errors.append(f"resolution 非法: {resolution}")

    width = int(manifest.get("width", 0))
    height = int(manifest.get("height", 0))
    if width <= 0 or height <= 0:
        errors.append("width/height 必须为正")

    base = path.parent
    for key in ("pcd_file", "grid_file"):
        name = str(manifest.get(key, ""))
        relative = pathlib.PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"{key} 必须是 manifest 目录内的相对路径: {name}")
        elif not (base / name).is_file():
            errors.append(f"{key} 不存在: {base / name}")

    if not errors:
        grid_path = base / manifest["grid_file"]
        try:
            pgm_width, pgm_height, _ = read_pgm_header(grid_path)
            if (pgm_width, pgm_height) != (width, height):
                errors.append(
                    f"栅格尺寸 {pgm_width}x{pgm_height} 与 manifest {width}x{height} 不一致"
                )
        except ValueError as error:
            errors.append(str(error))

    if not args.skip_hash and not errors:
        for key, sha_key in (("pcd_file", "pcd_sha256"), ("grid_file", "grid_sha256")):
            actual = sha256_file(base / manifest[key])
            if actual != manifest[sha_key]:
                errors.append(f"{key} 哈希不匹配: {actual} != {manifest[sha_key]}")

    if errors:
        print("地图校验失败：", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(f"地图校验通过: {path} (version={manifest['version']}, {width}x{height})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
