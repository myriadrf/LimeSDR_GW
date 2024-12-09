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
git clone --recursive git@github.com:enjoy-digital/LimeSDR-Mini-v2_GW
cd LimeSDR-Mini-v2_GW
git checkout origin/litex
```

## Build gateware

Go back to *LimeSDR-Mini-v2_GW*. Scrip to use depends on target board:

### limesdr_mini_v1

```bash
# Build the Gateware
python -m boards.targets.limesdr_mini_v1 --build [--with-bios] [--with-spi-flash] [--load]
```

Where:
- `--with-bios` enables *LiteX bios* (requires more resources)
- `--with-spi-flash` enables SPI Flash support
- `--load` to write bitstream

### limesdr_mini_v2

```bash
# Build the Gateware
python -m boards.targets.limesdr_mini_v2 --build [--with-bios] [--with-spi-flash] [--load] [--write] [--toolchain=TOOLCHAIN]
```

Where:

- `TOOLCHAIN` may be **diamond** or **trellis** (default: **diamond**)
- `--with-bios` enables *LiteX bios*
- `--with-spi-flash` enables SPI Flash support (only working with **trellis** toolchain)
- `--load` to load the bitstream (Volatile memory)
- `--write` to write the bitstream (SPI Flash)

### limesdr_xtrx

```bash
# Build the Gateware
python -m boards.targets.limesdr_xtrx --build [--with-bios] [--load] [--write]
```

Where:

- `--with-bios` enables *LiteX bios*
- `--load` to load the bitstream (Volatile memory)
- `--write` to write the bitstream (SPI Flash)

## Tests

To have debug messages:

``` bash
litex_term /dev/ttyUSB2
```

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

# Difference Between `gateware/LimeDFB` and `gateware/LimeDFB_LiteX`

The `gateware/LimeDFB` directory is a submodule for the LimeDFB repository.

Unlike `limesdr_xtrx`, both `limesdr_mini` and `limesdr_mini_2.0` use SPI for
register access along with dedicated VHDL modules. However, in *LimeDFB_LiteX*,
some of these modules have been removed, and the registers have been directly
integrated into the corresponding modules.

The `fpgacfg` component remains largely similar to the original approach but has
been adapted to use LiteX scripts. A single shared instance of `fpgacfg` is passed
via the constructor to the modules that require access to its registers
(`lms7002`, `rx_path`, and `tx_path`). This approach avoids redundant definitions
of the same CSRs and simplifies configuration at the firmware level.

Some new modules have been added, because these are required for *limesdr_mini* targets
but not present in *LimeDFB* directory:
- *FT601* for everything related to FT601 communication interface.
- *delayf_ctrl* for lattice `DELAYF` control/calibration
- *general*: misc modules required by others modules
- *general_periphery*: GPIOs and LEDs access
- *self_test*: clock calibration

All of them comes from *LimeSDR_Mini-v2_GW* repository with LiteX wrappers.

## Changes in LMS7002

1. **VHDL-to-LiteX Replacements**
   - **`lms7002_ddin.vhd` and `lms7002_ddout.vhd`**: These modules have been replaced with LiteX scripts of the same name.
   - **`lms7002_top.vhd`**: This module has been removed, and its associated logic has been fully integrated into `lms7002_top.py`.

2. **New Additions and Modifications**
   - **`lms7002_clk.py`**: This script now handles all logic related to clock management for the LMS7002.
   - **`test_data_dd`**: This component is present in *LimeDFB_LiteX* but has no equivalent in *LimeDFB*.
   - **`smpl_cmp.vhd`**: The *LimeDFB_LiteX* version of `smpl_cmp.vhd` is kept
     for `limesdr_mini` because the *LimeDFB* version requires two channels. To support
     both SISO and MIMO modes, the logic must be updated accordingly.
   - **`txiq_tst_ptrn.vhd`** The *LimeDFB_LiteX* version of this module is kept
     because, in *LimeDFB*, the test patterns are not configurable. **Question:** Is
     this configurability feature is used with `limesdr_mini`, or are the
     default pattern values always sufficient?

## Changes in `tx_path_top`

1. **VHDL-to-LiteX Replacements**
    - **`tx_path_top.vhd`**: This module has been removed, and its associated logic has been fully integrated into `tx_path_top.py`

2. **New Additions and Modifications**
  - `pct2data_buf_rd.vhd` and `pct2data_buf_wr.vhd`: These modules have been modified to use
     an `std_logic_vector` for the data bus instead of a complex type. Additionally, logic for selecting
     a FIFO has been moved to the `tx_path_top.py` level.
  - `sample_unpack.vhd` has been modified to support conversion by `GHDL`.

