#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""节点参数三处一致性校验（审计 2026-10-27 B3）。

`README.md` 是本仓唯一需求来源，但当时的参数表与实现脱节：11 个参数名在源码里
根本不存在，`base_frame` 的默认值也写错，下游照文档写配置必然启动失败。本脚本把
「源码 declare_parameter → README 参数表 → bringup/config/*.yaml」固化成可执行断言：

1. 每个节点的 README 参数名集合必须与 `declare_parameter` 名单**完全相等**；
2. 兵种配置里出现的键必须都已声明（拼错/改名前遗留的键立即失败）；
3. 兵种配置必须覆盖「非 launch 提供」的全部参数，`LAUNCH_PROVIDED` 里的名字由
   launch 的 LaunchArgument 注入，配置里出现属覆盖、不出现也合法。

用法：
    python3 src/foray_localization_tools/scripts/check_param_consistency.py
    python3 src/foray_localization_tools/scripts/check_param_consistency.py --root /path/to/repo

退出码：0 一致；1 存在不一致（逐条打印）。
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

# declare_parameter<T>("name", ...) / declare_parameter<T>("name")
DECLARE_RE = re.compile(r'declare_parameter<[^<>]+>\(\s*"([A-Za-z0-9_]+)"')

# 节点名 → 声明它的源文件（相对仓库根）
NODE_SOURCES = {
    "foray_localization": "src/foray_localization_ros/src/localization_node.cpp",
    "foray_map_server": "src/foray_localization_ros/src/map_server_node.cpp",
}

README_PATH = "README.md"
LAUNCH_PATH = "src/foray_localization_bringup/launch/localization.launch.py"
CONFIG_GLOB = "src/foray_localization_bringup/config/*.yaml"

# 由 launch 的 common_params 注入、启动时可被 LaunchArgument 覆盖的参数。
# 改动 launch/localization.launch.py 的注入集合时必须同步这里（下面会核对）。
LAUNCH_PROVIDED = frozenset({"map_manifest", "verify_map_hashes"})

# README 里每个节点参数表的标题形如「#### 本地化节点参数（`foray_localization`）」
HEADING_RE = re.compile(r"^(#{2,6})\s+(.*)$")
CELL_NAME_RE = re.compile(r"^`([A-Za-z0-9_]+)`$")
NODE_NAME_RE = re.compile(r"`([A-Za-z0-9_]+)`")


def parse_declared_params(source_text):
    """从节点源码中取出 declare_parameter 的参数名（保持出现顺序去重）。"""
    names = []
    for name in DECLARE_RE.findall(source_text):
        if name not in names:
            names.append(name)
    return names


def parse_readme_param_tables(readme_text):
    """取出 README 里「标题含 `节点名`」下的参数表首列。

    返回 {节点名: [参数名, ...]}；只把「首列整体是一个反引号标识符」的行算表项，
    因此说明文字、多参数合并行不会被误当成参数名。
    """
    tables = {}
    current_node = None
    for line in readme_text.splitlines():
        heading = HEADING_RE.match(line)
        if heading:
            level = len(heading.group(1))
            text = heading.group(2)
            node = NODE_NAME_RE.search(text)
            current_node = (
                node.group(1)
                if level >= 3 and node and node.group(1) in NODE_SOURCES
                else None
            )
            if current_node is not None:
                tables.setdefault(current_node, [])
            continue
        if current_node is None or not line.startswith("|"):
            continue
        cell = line.split("|")[1].strip()
        match = CELL_NAME_RE.match(cell)
        if match and match.group(1) not in tables[current_node]:
            tables[current_node].append(match.group(1))
    return tables


def load_config_keys(path):
    """读取一份 bringup 配置的 {节点名: set(参数名)}；缺 foray_localization 段即视为异常。"""
    import yaml  # 只在需要时导入，--help 无需 PyYAML

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    keys = {}
    for node, section in data.items():
        params = (section or {}).get("ros__parameters", {}) or {}
        keys[node] = set(params)
    return keys


def check(root):
    """返回不一致描述列表；空列表表示三处一致。"""
    root = pathlib.Path(root)
    failures = []

    declared = {}
    for node, relative in NODE_SOURCES.items():
        source = root / relative
        if not source.is_file():
            failures.append(f"找不到节点源码 {relative}")
            declared[node] = []
            continue
        declared[node] = parse_declared_params(source.read_text(encoding="utf-8"))
        if not declared[node]:
            failures.append(f"{relative} 里没有解析到任何 declare_parameter")

    readme = root / README_PATH
    if not readme.is_file():
        failures.append(f"找不到 {README_PATH}")
        documented = {}
    else:
        documented = parse_readme_param_tables(readme.read_text(encoding="utf-8"))

    for node, names in declared.items():
        declared_set = set(names)
        doc_set = set(documented.get(node, []))
        if node not in documented:
            failures.append(f"README 缺少节点 {node} 的参数表（标题需含 `{node}`）")
            continue
        for name in sorted(declared_set - doc_set):
            failures.append(f"README 缺参数 `{name}`（{node} 已声明）")
        for name in sorted(doc_set - declared_set):
            failures.append(
                f"README 多参数 `{name}`（{node} 未声明，文档会误导下游配置）"
            )

    launch = root / LAUNCH_PATH
    if not launch.is_file():
        failures.append(f"找不到 {LAUNCH_PATH}")
    else:
        launch_text = launch.read_text(encoding="utf-8")
        for name in sorted(LAUNCH_PROVIDED):
            if f'"{name}"' not in launch_text:
                failures.append(f"launch 未注入 `{name}`，但它被当作 launch 提供的参数")

    for node, names in declared.items():
        missing = LAUNCH_PROVIDED - set(names)
        for name in sorted(missing):
            failures.append(f"launch 注入的 `{name}` 未被 {node} 声明")

    configs = sorted(root.glob(CONFIG_GLOB))
    if not configs:
        failures.append(f"没有匹配到任何兵种配置：{CONFIG_GLOB}")
    for config in configs:
        label = config.relative_to(root)
        keys_by_node = load_config_keys(config)
        if "foray_localization" not in keys_by_node:
            failures.append(f"{label} 缺少 foray_localization 段")
        for node, keys in keys_by_node.items():
            if node not in declared:
                failures.append(f"{label} 出现未知节点 {node}")
                continue
            declared_set = set(declared[node])
            for name in sorted(keys - declared_set):
                failures.append(f"{label} 的 {node} 出现未声明参数 `{name}`")
            required = declared_set - LAUNCH_PROVIDED
            for name in sorted(required - keys):
                failures.append(
                    f"{label} 的 {node} 缺参数 `{name}`（非 launch 提供，必须在配置里给值）"
                )

    return failures


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="校验节点参数在源码/README/bringup 配置三处一致"
    )
    parser.add_argument(
        "--root", default=str(REPO_ROOT), help="仓库根目录（默认按脚本位置推断）"
    )
    args = parser.parse_args(argv)

    failures = check(args.root)
    if failures:
        print("参数一致性校验失败：", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        print(
            "修复方向：以实现为准同步 README 参数表与 bringup/config/*.yaml"
            "（README 是本仓唯一需求来源，改动须三处同时落地）。",
            file=sys.stderr,
        )
        return 1

    print(
        "参数一致："
        + " / ".join(
            f"{node} {count} 项" for node, count in _declared_counts(args.root).items()
        )
    )
    return 0


def _declared_counts(root):
    """成功路径的概要信息：各节点参数个数。"""
    root = pathlib.Path(root)
    return {
        node: len(parse_declared_params((root / relative).read_text(encoding="utf-8")))
        for node, relative in NODE_SOURCES.items()
    }


if __name__ == "__main__":
    sys.exit(main())
