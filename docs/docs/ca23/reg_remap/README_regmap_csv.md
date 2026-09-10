# Register Map CSV Workflow (CA23)

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
make regmap-ca23
```

Output kept in repo:

- `ca23_regremap_from_csv.rst`

Temporary merged CSVs (auto-generated, not kept):

- `/tmp/regmap/ca23_regmap_modules.csv`
- `/tmp/regmap/ca23_regmap_registers.csv`
- `/tmp/regmap/ca23_regmap_bitfields.csv`
