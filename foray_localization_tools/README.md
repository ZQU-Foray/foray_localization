# foray_localization_tools

离线地图工具。全部脚本只依赖 Python 3 与 PyYAML，不依赖 ROS 运行时。

## make_map.py

把 PCD 点云投影成 2D 占据栅格，并生成 manifest.yaml。

```bash
python3 scripts/make_map.py \
  --pcd /path/to/field.pcd \
  --output-dir /path/to/map_dir \
  --version rmul-2027-3v3-v1 \
  --resolution 0.05 \
  --field-size 12 8 \
  --center 0 0 \
  --z-min 0.10 --z-max 2.00
```

规则（与 `foray_localization_map` 一致）：

- 地图原点 = 场地几何中心，`origin` 是栅格 (0,0) 单元的世界坐标；
- 0 为占据、128 为未知；PGM 行 0 为顶部，写出时翻转 Y；
- manifest 中的 `pcd_sha256` / `grid_sha256` 由脚本计算，供节点加载时校验。

## validate_map.py

```bash
python3 scripts/validate_map.py /path/to/map_dir/manifest.yaml
```

校验必填字段、`frame_id == map`、分辨率范围、相对路径、栅格尺寸与 SHA-256。
CI 与上机前都应运行；失败返回非零退出码。

## 仿真闭环与评估

`gen_demo_field.py` → `make_map.py` → `sim_replay.py --dump-dir` →
`accumulate_pcd.py` → `make_map.py` 可离线复现整条链路，详见
[`docs/simulation.md`](../docs/simulation.md)。

- `sim_common.py`：纯 Python 几何、PCD 读写、射线投射、漂移模型、ATE/RPE；
- `sim_replay.py`：`--ros` 发话题，`--dump-dir` 写离线数据；
- `eval_trajectory.py`：ATE/RPE/OK 占比/状态迁移，超阈值 `exit 1`；
- `accumulate_pcd.py`：按真值位姿累积点云建图。

## 测试

```bash
python3 -m pip install -r test/requirements.txt
python3 -m pytest test -q
```

测试只用 pytest，不需要 ROS 运行时；`conftest.py` 负责把 `scripts/` 与
`foray_localization_sim/scripts/` 加入 `sys.path`。
