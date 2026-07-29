# Register Map CSV Workflow (LimeSDR USB)

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
make regmap-usb
```

Output kept in repo:

- `usb_regremap_from_csv.rst`

Temporary merged CSVs (auto-generated, not kept):

- `/tmp/regmap/usb_regmap_modules.csv`
- `/tmp/regmap/usb_regmap_registers.csv`
- `/tmp/regmap/usb_regmap_bitfields.csv`
