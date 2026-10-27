# Copyright 2026 ZQU-Foray
"""参数一致性校验脚本的自检：真实仓库必须通过，漂移必须被抓住。

脚本本身是「README 参数表 ↔ declare_parameter ↔ bringup 配置」的唯一裁判，
所以这里既跑仓库真身，也用合成仓库证明它在三类漂移上都会失败。
"""

import pathlib

import check_param_consistency as cpc
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]

LOCALIZATION_DECLARES = """\
namespace demo {
void Node::load() {
    map_manifest_ = declare_parameter<std::string>("map_manifest", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    verify_map_hashes_ = declare_parameter<bool>("verify_map_hashes", true);
    health_config_.ok_inlier_ratio = declare_parameter<double>("ok_inlier_ratio", 0.55);
}
} // namespace demo
"""

MAP_SERVER_DECLARES = """\
namespace demo {
void Node::load() {
    const auto manifest = declare_parameter<std::string>("map_manifest", "");
    const bool verify = declare_parameter<bool>("verify_map_hashes", true);
    const auto rate = declare_parameter<double>("publish_rate_hz", 1.0);
}
} // namespace demo
"""

LAUNCH_SOURCE = """\
def generate_launch_description():
    common_params = [
        config_file,
        {"map_manifest": map_manifest, "verify_map_hashes": verify_map_hashes},
    ]
"""

GOOD_README = """\
# demo

#### 本地化节点参数（`foray_localization`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_manifest` | `""` | 地图 |
| `map_frame` | `map` | 坐标系 |
| `verify_map_hashes` | `true` | 校验 |
| `ok_inlier_ratio` | `0.55` | 内点率 |

#### 地图服务节点参数（`foray_map_server`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_manifest` | `""` | 地图 |
| `verify_map_hashes` | `true` | 校验 |
| `publish_rate_hz` | `1.0` | 频率 |
"""

GOOD_CONFIG = """\
foray_map_server:
  ros__parameters:
    publish_rate_hz: 1.0
    verify_map_hashes: true

foray_localization:
  ros__parameters:
    map_frame: map
    ok_inlier_ratio: 0.55
"""


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _make_repo(tmp_path, readme=GOOD_README, config=GOOD_CONFIG):
    """按最小可校验结构造一个假仓库。"""
    _write(tmp_path / cpc.NODE_SOURCES["foray_localization"], LOCALIZATION_DECLARES)
    _write(tmp_path / cpc.NODE_SOURCES["foray_map_server"], MAP_SERVER_DECLARES)
    _write(tmp_path / cpc.LAUNCH_PATH, LAUNCH_SOURCE)
    _write(tmp_path / cpc.README_PATH, readme)
    _write(tmp_path / "src/foray_localization_bringup/config/sentry.yaml", config)
    return tmp_path


def test_repo_params_consistent():
    """本仓真身必须零不一致：这条断言就是 CI 的守门人。"""
    assert cpc.check(ROOT) == []


def test_parse_declared_params_keeps_order():
    assert cpc.parse_declared_params(LOCALIZATION_DECLARES) == [
        "map_manifest",
        "map_frame",
        "verify_map_hashes",
        "ok_inlier_ratio",
    ]


def test_readme_parser_handles_multi_node_tables():
    tables = cpc.parse_readme_param_tables(GOOD_README)
    assert tables["foray_localization"] == [
        "map_manifest",
        "map_frame",
        "verify_map_hashes",
        "ok_inlier_ratio",
    ]
    assert tables["foray_map_server"] == [
        "map_manifest",
        "verify_map_hashes",
        "publish_rate_hz",
    ]


def test_synthetic_repo_passes(tmp_path):
    assert cpc.check(_make_repo(tmp_path)) == []


def test_detects_documented_param_that_does_not_exist(tmp_path):
    """审计 B3 的原始病：README 写了实现里没有的参数名。"""
    readme = GOOD_README.replace(
        "| `map_frame` | `map` | 坐标系 |",
        "| `map_frame` | `map` | 坐标系 |\n| `voxel_size` | `0.10` | 已改名 |",
    )
    failures = cpc.check(_make_repo(tmp_path, readme=readme))
    assert any("README 多参数 `voxel_size`" in item for item in failures)


def test_detects_declared_param_missing_from_readme(tmp_path):
    readme = GOOD_README.replace("| `ok_inlier_ratio` | `0.55` | 内点率 |\n", "")
    failures = cpc.check(_make_repo(tmp_path, readme=readme))
    assert any("README 缺参数 `ok_inlier_ratio`" in item for item in failures)


def test_detects_config_param_that_does_not_exist(tmp_path):
    config = GOOD_CONFIG.replace(
        "    map_frame: map", "    map_frame: map\n    cluster_tolerance: 0.3"
    )
    failures = cpc.check(_make_repo(tmp_path, config=config))
    assert any("未声明参数 `cluster_tolerance`" in item for item in failures)


def test_detects_config_missing_required_param(tmp_path):
    config = GOOD_CONFIG.replace("    ok_inlier_ratio: 0.55\n", "")
    failures = cpc.check(_make_repo(tmp_path, config=config))
    assert any("缺参数 `ok_inlier_ratio`" in item for item in failures)


def test_main_returns_nonzero_on_drift(tmp_path, capsys):
    readme = GOOD_README.replace(
        "| `map_frame` | `map` | 坐标系 |",
        "| `map_frame` | `map` | 坐标系 |\n| `registration_rate_hz` | `5.0` | 旧名 |",
    )
    repo = _make_repo(tmp_path, readme=readme)
    assert cpc.main(["--root", str(repo)]) == 1
    assert "参数一致性校验失败" in capsys.readouterr().err


def test_main_returns_zero_when_consistent(tmp_path, capsys):
    assert cpc.main(["--root", str(_make_repo(tmp_path))]) == 0
    assert "参数一致" in capsys.readouterr().out


@pytest.mark.parametrize("node", sorted(cpc.NODE_SOURCES))
def test_every_node_has_readme_table(node):
    tables = cpc.parse_readme_param_tables(
        (ROOT / cpc.README_PATH).read_text(encoding="utf-8")
    )
    assert node in tables
