# Building the gateware / working with the firmware

Here we assume LiteX, diamond and a riscv toolchain are already
present/installed.

## Advantages of LiteX Gateware over previous Gateware.

### Current
- Simplified integration/cores interconnection.
- Selection of possible various CPUs with --cpu-type (VexRiscv, PicoRV32, etc...) to select tradeoff between performance/resoruce usage.
- Use of LiteX Peripherals.
- Use of LiteX CSR Bus to simplify peripheral control from CPU.
- Observation/Debug capabilities with LiteScope Logic Analyser over UART/JTAG.
- Simplified firmware build/integration (automatic).
- Simplified firmware development with serialboot of firmware not requiring full Gateware recompilation.

### Future
- Once all FIFOs will be replaced, possibility to try GHDL/Yosys/NextPnr build.
- Possibility to use the same codebase/project between LimeSDR Mini and Mini-V2 (with just minor changes).
- Possibility to share the same codebase for common peripherals between LimeSDR Mini, XTRX and future boards.

## LimeSDR_GW / LimeDFB hash

Before 20241204:
- LimeSDR_GW: 8348e184ccfcbbb79c738188cf43e4b0b3deca6e
- LimeDFB : 3ad6d470286fd857e3bc7fef2dbc9fe59d8c65c3

After 20241204:
- LimeSDR_GW: 1837abce83e30d7f35db15b03c30c6b43c08ec97
- LimeDFB : 05abc7eb57875a92de8018cbb814006bfb5e0a38

## Fetch repository

Fetch this repository and switch to development branch:

```bash
git clone git@github.com:enjoy-digital/LimeSDR-Mini-v2_GW
cd LimeSDR-Mini-v2_GW
git checkout origin/litex
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
