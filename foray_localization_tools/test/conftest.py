"""让 pytest 能直接导入 scripts/ 下的纯 Python 工具。"""

import pathlib
import sys

SCRIPT_DIRS = [
    pathlib.Path(__file__).resolve().parents[1] / "scripts",
    pathlib.Path(__file__).resolve().parents[2] / "foray_localization_sim" / "scripts",
]
for directory in SCRIPT_DIRS:
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
