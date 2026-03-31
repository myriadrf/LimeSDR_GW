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
	@python3 tools/regmap_csv_to_rst.py \
		--modules-csv docs/limesdr-xtrx/reg_remap/xtrx_regmap_modules.csv \
		--registers-csv docs/limesdr-xtrx/reg_remap/xtrx_regmap_registers.csv \
		--bitfields-csv docs/limesdr-xtrx/reg_remap/xtrx_regmap_bitfields.csv \
		--output docs/limesdr-xtrx/reg_remap/xtrx_regremap_from_csv.rst \
		--title "LimeSDR XTRX Host Register Reference"

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
