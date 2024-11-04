# Building the gateware / working with the firmware

Here we assume LiteX, diamond and a riscv toolchain are already
present/installed.

## Fetch repository

Fetch this repository and switch to development branch:

```bash
git clone git@github.com:enjoy-digital/LimeSDR-Mini-v2_GW
cd LimeSDR-Mini-v2_GW
git checkout origin/litex_SoC
```

## Update LiteX

LiteX repository must be updated to support some not mainlined features:

```bash
cd /somewhere/litex
patch -p1 < /somewhere/LimeSDR-Mini-v2_GW/litex_diamond_additions.patch
```

## Build gateware

Go back to *LimeSDR-Mini-v2_GW* and execute script with:

```bash
cd /somewhere/LimeSDR-Mini-v2_GW
# Build the Gateware
./limesdr_mini_v2.py --build --toolchain=diamond
# Load bitstream
openFPGALoader -c ft2232
build/limesdr_mini_v2_platform/gateware/limesdr_mini_v2_platform_impl.bit
```

To have debug messages:

``` bash
litex_term /dev/ttyUSB2
```

## Tests

Tests are done using **GQRX**:

![gqrx_conf_box](https://github.com/user-attachments/assets/fc741ce4-d149-421d-923d-bc621111c3f5)

## Firmware modifications

*firmware* directory contains firmware executed by the SoC and loaded
automatically by the LiteX bios. It's possible to modify the code and to reload
this new version without having to rebuild the gateware:

In a first terminal:

```bash
cd firmware
# do some modifications
make
# Force SoC reset
openFPGALoader -c ft2232
../build/limesdr_mini_v2_platform/gateware/limesdr_mini_v2_platform_impl.bit
```

In a second terminal:

```bash
litex_term /dev/ttyUSB2 --kernel firmware/firmware.bin
```

### Firmware content

- `i2c0.c` and `i2c0.h` handles i2c protocol between the SoC and *LM75*
- `csr_access.h` and `csr_access.c` handles access to *fpgacfg*, *tstcfg*,
  *periphcfg* and *pllcfg*
  - `main.c` is a main part with function to read/write LMS7002 registers and
    DAC.
