# Register Map CSV Workflow (LimeSDR Mini v2)

This board uses shared Mini-family base host-register CSVs from:

- `docs/common_host_regs/modules.csv`
- `docs/common_host_regs/registers.csv`
- `docs/common_host_regs/bitfields.csv`

Board-specific differences go to:

- `modules_override.csv`
- `registers_override.csv`
- `bitfields_override.csv`

Generate merged CSVs and RST:

```bash
make regmap-mini-v2
```

Output kept in repo:

- `mini_v2_regremap_from_csv.rst`

Temporary merged CSVs (auto-generated, not kept):

- `/tmp/regmap/mini_v2_regmap_modules.csv`
- `/tmp/regmap/mini_v2_regmap_registers.csv`
- `/tmp/regmap/mini_v2_regmap_bitfields.csv`
