# Register Map CSV Workflow (LimeSDR Mini v1)

Use these CSV files as the source of truth, then generate RST:

- `mini_v1_regmap_modules.csv`
- `mini_v1_regmap_registers.csv`
- `mini_v1_regmap_bitfields.csv`

Generate:

```bash
python3 tools/regmap_csv_to_rst.py \
  --modules-csv docs/limesdr-mini-v1/reg_remap/mini_v1_regmap_modules.csv \
  --registers-csv docs/limesdr-mini-v1/reg_remap/mini_v1_regmap_registers.csv \
  --bitfields-csv docs/limesdr-mini-v1/reg_remap/mini_v1_regmap_bitfields.csv \
  --output docs/limesdr-mini-v1/reg_remap/mini_v1_regremap_from_csv.rst \
  --title "LimeSDR Mini V1 Host Register Reference" \
  --label-prefix mini_v1
```

Or use:

```bash
make regmap-mini-v1
```
