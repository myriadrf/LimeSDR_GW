# Common Host Register Map

This directory contains the shared base register map used by multiple boards.

Base CSV files:
- `modules.csv`
- `registers.csv`
- `bitfields.csv`

Board-specific differences should be added in each board's `reg_remap/*_override.csv` files.
Generated board CSVs are produced by `tools/regmap_merge_csv.py`.

Boards with non-matching maps (for example LimeSDR Mini v1/v2) use override files to adapt this base.
