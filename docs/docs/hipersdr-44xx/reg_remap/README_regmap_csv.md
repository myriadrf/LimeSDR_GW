# Register Map CSV Workflow (HiperSDR 44xx)

This board uses shared base host-register CSVs from:

- `docs/common_host_regs/modules.csv`
- `docs/common_host_regs/registers.csv`
- `docs/common_host_regs/bitfields.csv`

Board-specific differences go to:

- `modules_override.csv`
- `registers_override.csv`
- `bitfields_override.csv`

Generate merged CSVs and RST:

```bash
make regmap-hipersdr-44xx
```

Output kept in repo:

- `hipersdr_44xx_regremap_from_csv.rst`

Temporary merged CSVs (auto-generated, not kept):

- `/tmp/regmap/hipersdr_44xx_regmap_modules.csv`
- `/tmp/regmap/hipersdr_44xx_regmap_registers.csv`
- `/tmp/regmap/hipersdr_44xx_regmap_bitfields.csv`
