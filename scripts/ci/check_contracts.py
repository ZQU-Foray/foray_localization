#!/usr/bin/env python3
"""契约快照门禁：msg / srv / action 接口不得静默变更。

存在的理由很具体：下游仓按 msg 字段写配置与回调，接口改一个字段名，
下游是**编译期报错**还是**运行期静默错位**，取决于改的人有没有想起来通知别人。
快照把「有没有想起来」变成「diff 里有没有出现 contracts.sha256」。

设计取舍：
- 快照用**标准 sha256sum 格式**，所以即使本脚本坏了，`sha256sum -c contracts.sha256`
  依然能校验，不把整条链路押在一个自研脚本上。
- 只断言「内容与快照一致」，不做兼容性判断——兼容性是人审的事，
  机器只负责保证「变更一定被看见」。

用法：
    python3 scripts/ci/check_contracts.py            # 校验，不一致 exit 1
    python3 scripts/ci/check_contracts.py --update   # 改完接口后重生成快照
"""

from __future__ import annotations

import hashlib
import re
import subprocess
import sys
from pathlib import Path

SNAPSHOT = Path("contracts.sha256")
CONTRACT_SUFFIXES = (".msg", ".srv", ".action")

SNAPSHOT_HEADER = """\
# 契约快照：msg / srv / action 接口指纹，由 scripts/ci/check_contracts.py 维护。
#
# 改动任何一个接口文件后，必须在本提交内执行：
#     python3 scripts/ci/check_contracts.py --update
# 并在同一提交的说明里写明兼容性影响（新增字段 / 改名 / 删字段 / 改默认值）。
#
# 这行快照的唯一职责，是让「接口变了但没人通知下游」在 PR diff 里必然可见。
# 校验：python3 scripts/ci/check_contracts.py   （或 sha256sum -c contracts.sha256）
"""


def contract_files() -> list[Path]:
    """返回全部接口文件的仓根相对路径，按路径排序保证快照稳定。"""
    result = subprocess.run(
        ["git", "ls-files"],
        capture_output=True,
        text=True,
        check=True,
    )
    files = [
        Path(line)
        for line in result.stdout.splitlines()
        if line.endswith(CONTRACT_SUFFIXES) and line
    ]
    return sorted(files)


def digest(path: Path) -> str:
    """返回文件的 sha256 十六进制摘要（按字节，不做换行规范化）。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def render(files: list[Path]) -> str:
    """渲染快照正文。"""
    lines = [SNAPSHOT_HEADER]
    for path in files:
        lines.append(f"{digest(path)}  {path.as_posix()}\n")
    return "".join(lines)


def parse_snapshot(text: str) -> dict[str, str]:
    """解析 sha256sum 格式快照，忽略注释与空行。"""
    entries: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        match = re.fullmatch(r"([0-9a-fA-F]{64})\s+\*?(.+)", stripped)
        if match is None:
            raise ValueError(f"快照行无法解析：{line!r}")
        entries[match.group(2).strip()] = match.group(1).lower()
    return entries


def main(argv: list[str]) -> int:
    files = contract_files()
    if not files:
        print("未发现 msg/srv/action 文件，跳过契约快照校验。")
        return 0

    actual = {path.as_posix(): digest(path) for path in files}

    if "--update" in argv:
        SNAPSHOT.write_text(render(files), encoding="utf-8")
        print(f"已更新 {SNAPSHOT}：{len(files)} 个契约文件。")
        return 0

    if not SNAPSHOT.is_file():
        print(
            f"::error::缺少 {SNAPSHOT}，请执行 python3 scripts/ci/check_contracts.py --update",
            file=sys.stderr,
        )
        return 1

    expected = parse_snapshot(SNAPSHOT.read_text(encoding="utf-8"))

    added = sorted(set(actual) - set(expected))
    removed = sorted(set(expected) - set(actual))
    changed = sorted(
        path for path in set(actual) & set(expected) if actual[path] != expected[path]
    )

    if not (added or removed or changed):
        print(f"契约快照校验通过：{len(files)} 个接口文件未变更。")
        return 0

    for path in added:
        print(f"::error::新增契约文件未登记：{path}")
    for path in removed:
        print(f"::error::契约文件已删除但快照仍在：{path}")
    for path in changed:
        print(f"::error::契约内容变更未同步快照：{path}")

    print(
        "\n接口发生了变更。这是**预期内**的动作，只要它是有意的：\n"
        "  1. 在提交说明里写明兼容性影响（新增字段 / 改名 / 删字段 / 改默认值）；\n"
        "  2. 执行 python3 scripts/ci/check_contracts.py --update；\n"
        "  3. 把 contracts.sha256 的变更一并提交，让同步改动出现在同一个 diff 里。",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
