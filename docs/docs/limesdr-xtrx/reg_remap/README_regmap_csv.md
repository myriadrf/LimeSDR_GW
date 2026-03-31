# Register Map CSV Workflow (XTRX)

Shared base CSV source:

- `docs/common_host_regs/modules.csv`
- `docs/common_host_regs/registers.csv`
- `docs/common_host_regs/bitfields.csv`

Board-specific overrides in this folder:

- `modules_override.csv`
- `registers_override.csv`
- `bitfields_override.csv`

Generate:

```bash
make regmap-xtrx
```

Output kept in repo:

- `xtrx_regremap_from_csv.rst`

Temporary merged CSVs (auto-generated, not kept):

- `/tmp/regmap/xtrx_regmap_modules.csv`
- `/tmp/regmap/xtrx_regmap_registers.csv`
- `/tmp/regmap/xtrx_regmap_bitfields.csv`
