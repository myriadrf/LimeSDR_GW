# Register Map CSV Workflow

Use these CSV files as the source of truth, then generate RST:

- `xtrx_regmap_modules.csv`
- `xtrx_regmap_registers.csv`
- `xtrx_regmap_bitfields.csv`

Generate:

```bash
make regmap-xtrx
```

Output:

- `docs/xtrx_regremap_from_csv.rst`

## CSV Schemas

### 1) modules

File: `xtrx_regmap_modules.csv`

Columns:

- `module`
- `address_start`
- `address_end`
- `typical_use`

Example:

```csv
module,address_start,address_end,typical_use
FPGACFG,0x0000,0x001F,"Board ID/revision, stream mode"
```

### 2) registers

File: `xtrx_regmap_registers.csv`

Columns:

- `module`
- `address`
- `address_end` (optional; empty means single address)
- `default`
- `name`
- `description`

Example:

```csv
module,address,address_end,default,name,description
FPGACFG,0x0007,,0x0003,ch_en,"Channel enable register."
```

### 3) bitfields

File: `xtrx_regmap_bitfields.csv`

Columns:

- `address`
- `title`
- `access`
- `default`
- `field`
- `msb`
- `lsb`
- `values`
- `description`

Rules:

- One row per field.
- For the same register address, you can leave `title/access/default` empty after the first row.
- `msb`/`lsb` are numeric bit indices (for example `14`, `3`).
- Every non-reserved single-address register in `registers.csv` must have at least one matching
  row in `bitfields.csv` (same `address`).
- Reserved single-address registers are auto-filled as `[15:0] reserved` if omitted.

Example:

```csv
address,title,access,default,field,msb,lsb,values,description
0x0007,ch_en,R/W,0x0003,ch_en,1,0,"01=A,10=B,11=A+B","Channel selection."
```

## Notes

- Quote any CSV value that contains commas.
- Bitfield WaveDrom diagrams are generated automatically (16-bit, 2 rows of 8 bits).
