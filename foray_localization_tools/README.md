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
