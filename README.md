# Building the gateware / working with the firmware

Here we assume LiteX, diamond and a riscv toolchain are already
present/installed.

## Advantages of LiteX Gateware over previous Gateware.

### Current
- Simplified integration/cores interconnection.
- Selection of possible various CPUs with --cpu-type (VexRiscv, PicoRV32, etc...) to select tradeoff between performance/resource usage.
- Use of LiteX Peripherals.
- Use of LiteX CSR Bus to simplify peripheral control from CPU.
- Observation/Debug capabilities with LiteScope Logic Analyser over UART/JTAG.
- Simplified firmware build/integration (automatic).
- Simplified firmware development with serialboot of firmware not requiring full Gateware recompilation.

## Questions

1. **Use of Single-Bit Registers**
   Why are registers split into one-bit registers instead of using `CSRField`?
   - Would it be possible to re-use the *limesdr_mini* memory map for all boards?
   - Some CSR, for the same register, are declared in more than one module. Looks
     better to have a common instance shared between modules.
2. **Reset Signal Differences**
   The reset signals for *limesdr_mini* and *limesdr_xtrx* appear to be different.
   - Is there a way to standardize or converge them?
3. **Clock Domains Complexity**
   Currently, there are three clock domains for `tx_path` and `rx_path`.
   - Why not simplify this by using a CDC (Clock Domain Crossing) in the
     communication interface module and a second CDC in `lms7002_top` to bridge
     the `sys` clock with `lms_tx`/`lms_rx`?
4. **Tests**
   Unlike *limesdr_xtrx*, *limesdr_mini* have a *test_data_dd.vhd* module and,
   for Rx and Tx (*lms7002* module) a mux with more options. Is this test remains
   relevant or may be removed to align all 3 boards?
5. **VCTCXO_TAMER**
   Why not using LiteUART instead of a VHDL implementation?

## Status

### **Code / Repository Status**
- **Vendor FIFOs Removed**: All vendor FIFOs have been replaced by LiteX FIFOs.
- **Unified Codebase**: A single codebase is now used for all three boards.
  Board-specific functionality is conditionally included at build time based on
  the target name.
- **Verilog Conversion**: Code from the *LimeSDR_Mini-V2_GW* repository can be
  converted to Verilog using *GHDL* (default for `limesdr_mini_v2`).
- **Build Tools**:
  - **limesdr_mini_v2**: Can be built using either *Diamond* or *Yosys/nextPNR/Trellis*
    (diamond has some limitations: SPI Flash not working, no boot with VexRiscv).
  - **limesdr_mini_v1**: Requires *Quartus* for building.
  - **limesdr_xtrx**: Built using *Vivado*.
- **VexRiscv Support**: VexRiscv can be used as the firmware processor for all boards.
- **Conditional Primitives**: Specific code and hardware primitives are enabled
  or disabled at build time based on the target name.

### Boards Status

#### **limesdr_mini_v1**

- **Build Tool**: *Quartus*
- **Detection**: Board is correctly detected with `limeDevice -f`.
- **Rx Path**:
  - Validated with *GQRX* appimage (via *LimeSuite*).
  - Also tested with `FM_receiver.grc` using *GNURadio* + *LimeSuiteNG*.
- **Tx Path**:
  - Validated with `test_miniv2_tx.grc` using *GNURadio* + *LimeSuiteNG*.

**Limitations**:
- **PLL Calibration / Delays**: Not used. The LiteX implementation of the Altera PLL module needs improvement.
- **SPI Flash**: Not tested.

#### limesdr_mini_v2

- **Build Tool**: Can be built with either *Diamond* or *Yosys/nextPNR/Trellis*.
- **Detection**: Board is correctly detected with `limeDevice -f`.
- **Rx Path**:
  - Validated with *GQRX* appimage (via *LimeSuite*).
  - Also tested with `FM_receiver.grc` using *GNURadio* + *LimeSuiteNG*.
- **Tx Path**:
  - Validated with `test_miniv2_tx.grc` using *GNURadio* + *LimeSuiteNG*.

**Limitations**:
- **SPI Flash**:
  - The SPI Flash is detected successfully (a read at offset 0 returns the first bytes of the bitstream).
  - However, the logic in the firmware to manage the SPI Flash must be re-added.

#### limesdr_xtrx

- **Build Tool**: *Vivado*
- **Detection**:
  - The board is detected by the operating system (via `lspci`).
  - The board is correctly detected with `limeDevice -f`.

**Limitations**:
- **Rx Path**: Not working (no samples received).
- **Tx Path**: Not tested.

## potential future improvements

1. **Clock Domain Crossings (CDC)**
   - Not all CDCs are fully implemented or operational.
   - A potential first step toward improvement would be to simplify the CDC logic.
2. **Reset Signal Improvements**
   - The handling of `reset` signals needs improvement.
   - Currently, the `tx_en` or `rx_en` signals are used as resets, but the approach
     varies depending on the board.
   - A more unified and consistent reset strategy is required across all boards.
1. **Input/Output Data Stream Delays (limesdr_mini_v1)**
   - For *limesdr_mini_v1*, input and output data stream delays are not fully managed.
   - The LiteX PLL module must be enhanced to support dynamic phase configuration,
     which would allow better handling of data stream alignment.
2. **Support for limesdr_xtrx**
   - The *limesdr_xtrx* target is currently a proof of concept.
   - Full support is required to bring its status to parity with *LimeSDR_GW*,
     ensuring the same level of stability, feature set, and testing coverage.
3. **Firmware merge**
   - *limesdr_mini_v1* and *limesdr_mini_v2* have one firmware (`firmware_mini`),
     *limesdr_xtrx* has another firmware (`firmware_xtrx`). A common firmware, or
     at least a common codebase may simplify future evolutions
   - before merge memory map may need to be unified

## LimeSDR_GW / LimeDFB hash

Before 20241204:
- LimeSDR_GW: 8348e184ccfcbbb79c738188cf43e4b0b3deca6e
- LimeDFB : 3ad6d470286fd857e3bc7fef2dbc9fe59d8c65c3

Previous hash (20250109):
- LimeSDR_GW: ff0ba8b156cbd4e59dc63772738cd45d83156e19 (Mon Dec 16 16:24:03 2024 +0200)
- LimeDFB: a9295be5142a907f01d963cd3649e0fc5f610e56 (Mon Dec 16 15:30:52 2024 +0200)
- LimeSuiteNG: 7ea8555c8ef8a0002dd060a12664b238470fce71 (Wed Jan 8 16:09:40 2025 +0200)

Current hash (20250114):
- LimeSDR_GW: a541532e2c00f24d68fc0ff256d7bb16a2fd061e (Mon Jan 13 14:38:31 2025 +0200)
- LimeDFB: a9295be5142a907f01d963cd3649e0fc5f610e56 (Mon Dec 16 15:30:52 2024 +0200)
- LimeSuiteNG: 7ea8555c8ef8a0002dd060a12664b238470fce71 (Wed Jan 8 16:09:40 2025 +0200)

## Fetch repository

Fetch this repository and switch to development branch:

```bash
git clone --recursive git@github.com:enjoy-digital/LimeSDR-Mini-v2_GW -b litex
cd LimeSDR-Mini-v2_GW
```

## Build gateware

Go back to *LimeSDR-Mini-v2_GW*. Scrip to use depends on target board:

### limesdr_mini_v1

```bash
# Build the Gateware
python -m boards.targets.limesdr_mini_v1 --build [--with-bios] [--without-spi-flash] [--load]
```

Where:
- `--with-bios` enables *LiteX bios* (requires more resources)
- `--with-spi-flash` to disable SPI Flash support
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

As done for *limesdr_mini* `tx_path` and `rx_path` are not directly instanciated
at the target level but `rxtx_top.py` remains used to instanciates both submodules.

## Firmware

- Firmware for *limesdr_mini_v1* and *limesdr_mini_v2* is in *firmware_mini* directory.
  This firmware is an adapted version of *LimeSDR_Mini-v2_GW* software.
- *limesdr_xtrx* firmware is in *firmware_xtrx* repository. This firmware is an
- adapted version of the one in *LimeSDR_GW* repository.

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

## Changes in `rx_path_top`

1. **VHDL-To-LiteX Replacements**
   - `rx_path_top.vhd`: Most of the logic has been moved to `rx_path_top.py` only
     few process remains. **Question**: this logic may/must be converted to LiteX
2. **New Additions and Modifications**
   - `iq_stream_combiner.vhd` has been modified to correctly handle *SISO* mode


## Documentation build

```bash
pip3 install sphinx sphinx_rtd_theme sphinx_tabs myst_parser
cd docs
make html
```