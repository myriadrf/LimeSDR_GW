# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: regmap-xtrx
regmap-xtrx:
	@python3 tools/regmap_merge_csv.py \
		--common-dir docs/common_host_regs \
		--override-dir docs/limesdr-xtrx/reg_remap \
		--out-modules /tmp/regmap/xtrx_regmap_modules.csv \
		--out-registers /tmp/regmap/xtrx_regmap_registers.csv \
		--out-bitfields /tmp/regmap/xtrx_regmap_bitfields.csv
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv /tmp/regmap/xtrx_regmap_modules.csv \
		--registers-csv /tmp/regmap/xtrx_regmap_registers.csv \
		--bitfields-csv /tmp/regmap/xtrx_regmap_bitfields.csv \
		--output docs/limesdr-xtrx/reg_remap/xtrx_regremap_from_csv.rst \
		--title "LimeSDR XTRX Host Register Reference" \
		--label-prefix xtrx

.PHONY: regmap-hipersdr-44xx
regmap-hipersdr-44xx:
	@python3 tools/regmap_merge_csv.py \
		--common-dir docs/common_host_regs \
		--override-dir docs/hipersdr-44xx/reg_remap \
		--out-modules /tmp/regmap/hipersdr_44xx_regmap_modules.csv \
		--out-registers /tmp/regmap/hipersdr_44xx_regmap_registers.csv \
		--out-bitfields /tmp/regmap/hipersdr_44xx_regmap_bitfields.csv
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv /tmp/regmap/hipersdr_44xx_regmap_modules.csv \
		--registers-csv /tmp/regmap/hipersdr_44xx_regmap_registers.csv \
		--bitfields-csv /tmp/regmap/hipersdr_44xx_regmap_bitfields.csv \
		--output docs/hipersdr-44xx/reg_remap/hipersdr_44xx_regremap_from_csv.rst \
		--title "HiperSDR 44xx Host Register Reference" \
		--label-prefix hipersdr_44xx

.PHONY: regmap-ssdr-rev2
regmap-ssdr-rev2:
	@python3 tools/regmap_merge_csv.py \
		--common-dir docs/common_host_regs \
		--override-dir docs/ssdr_rev2/reg_remap \
		--out-modules /tmp/regmap/ssdr_rev2_regmap_modules.csv \
		--out-registers /tmp/regmap/ssdr_rev2_regmap_registers.csv \
		--out-bitfields /tmp/regmap/ssdr_rev2_regmap_bitfields.csv
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv /tmp/regmap/ssdr_rev2_regmap_modules.csv \
		--registers-csv /tmp/regmap/ssdr_rev2_regmap_registers.csv \
		--bitfields-csv /tmp/regmap/ssdr_rev2_regmap_bitfields.csv \
		--output docs/ssdr_rev2/reg_remap/ssdr_rev2_regremap_from_csv.rst \
		--title "sSDR rev2 Host Register Reference" \
		--label-prefix ssdr_rev2

.PHONY: regmap-all
regmap-all: regmap-xtrx regmap-hipersdr-44xx regmap-ssdr-rev2 regmap-mini-v1 regmap-mini-v2

.PHONY: regmap-mini-v1
regmap-mini-v1:
	@python3 tools/regmap_merge_csv.py \
		--common-dir docs/common_host_regs \
		--override-dir docs/limesdr-mini-v1/reg_remap \
		--out-modules /tmp/regmap/mini_v1_regmap_modules.csv \
		--out-registers /tmp/regmap/mini_v1_regmap_registers.csv \
		--out-bitfields /tmp/regmap/mini_v1_regmap_bitfields.csv
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv /tmp/regmap/mini_v1_regmap_modules.csv \
		--registers-csv /tmp/regmap/mini_v1_regmap_registers.csv \
		--bitfields-csv /tmp/regmap/mini_v1_regmap_bitfields.csv \
		--output docs/limesdr-mini-v1/reg_remap/mini_v1_regremap_from_csv.rst \
		--title "LimeSDR Mini V1 Host Register Reference" \
		--label-prefix mini_v1

.PHONY: regmap-mini-v2
regmap-mini-v2:
	@python3 tools/regmap_merge_csv.py \
		--common-dir docs/common_host_regs \
		--override-dir docs/limesdr-mini-v2/reg_remap \
		--out-modules /tmp/regmap/mini_v2_regmap_modules.csv \
		--out-registers /tmp/regmap/mini_v2_regmap_registers.csv \
		--out-bitfields /tmp/regmap/mini_v2_regmap_bitfields.csv
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv /tmp/regmap/mini_v2_regmap_modules.csv \
		--registers-csv /tmp/regmap/mini_v2_regmap_registers.csv \
		--bitfields-csv /tmp/regmap/mini_v2_regmap_bitfields.csv \
		--output docs/limesdr-mini-v2/reg_remap/mini_v2_regremap_from_csv.rst \
		--title "LimeSDR Mini V2 Host Register Reference" \
		--label-prefix mini_v2

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
