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

- **Build Tool**: *Quartus*, *srec_cat* to convert `firmware.bin` to
  `firmware.hex` (intel Format required by quartus to store firmware into *UFM
  sections*
- **Detection**: Board is correctly detected with `limeDevice -f`.
- **Rx Path**:
  - Validated with *GQRX* appimage (via *LimeSuite*).
  - Also tested with `FM_receiver.grc` using *GNURadio* + *LimeSuiteNG*.
- **Tx Path**:
  - Validated with `test_miniv2_tx.grc` using *GNURadio* + *LimeSuiteNG*.

**Limitations**:
- **PLL Calibration / Delays**: Not used. The LiteX implementation of the Altera PLL module needs improvement.
- The same bitstream can't be used for both Golden and operational: one bitstream must be build for Golden,
  and a second for operational. See section **LimeSDR Mini v1 Boot**.

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
3. **Input/Output Data Stream Delays (limesdr_mini_v1)**
   - For *limesdr_mini_v1*, input and output data stream delays are not fully managed.
   - The LiteX PLL module must be enhanced to support dynamic phase configuration,
     which would allow better handling of data stream alignment.

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
python -m boards.targets.limesdr_mini_v1 --build [--with-bios] [--with-spi-flash] [--golden]
```

Where:
- `--with-bios` enables *LiteX bios* (requires more resources)
- `--with-spi-flash` to disable SPI Flash support
- `--golden` to create the Golden bitstream (without RX/TX Path nor LMS7002 Modules to reduces
  size since it uses ROM)

Since the Operational bitstream executes firmware from internal flash, the `--load` option
is not supported.

To update the LimeSDR Mini v1, follow this sequence:

1. Build the Golden bitstream (if not already done) using the `--golden` option.
2. Build the Operational bitstream.
3. After the previous step, the `.rpd`, `.pof`, and `.svf` files will be available.

To write the new image:
- **Full update:**
  ```sh
  openFPGALoader -c [cable] LimeSDR-Mini_lms7_trx_HW_1.2.svf
  ```
- **Operational bitstream and firmware update only:**
  ```sh
  limeFLASH --device=Mini --target=FPGA/FLASH LimeSDR-Mini_lms7_trx_HW_1.2_auto.rpd
  ```

### limesdr_mini_v2

```bash
# Build the Gateware
python -m boards.targets.limesdr_mini_v2 --build [--with-bios] [--without-spi-flash] [--load] [--flash] [--flash-user] [--flash-golden] [--cable] [--toolchain=TOOLCHAIN]
```

Where:

- `TOOLCHAIN` may be **diamond** or **trellis** (default: **trellis**)
- `--with-bios` enables *LiteX bios*
- `--without-spi-flash` disables SPI Flash support (only working with **trellis** toolchain)
- `--load` to load the bitstream into volatile memory
- `--flash` writes the bitstream to SPI Flash
- `--flash-golden` flashes the golden bitstream (fallback) at address `0x00140000`
- `--flash-user` flashes the user bitstream (operational) at address `0x00280000`

#### Golden/Operational bitstreams

- The **Golden bitstream** is provided in the repository at `tools/limesdr_mini_v2_golden.bit`
- The **User bitstream** is built using the command mentionned above

After generating the User bitstream, the repository's root directory contains:
- `limesdr_mini_v2.bin`: A composite image containing both golden and user bitstreams. The FPGA will attempt to load
  the User bitstream first and fall back to the Golden bitstream if loading fails.
- `limesdr_mini_v2.mcs`: the same file as `limesdr_mini_v2.bin`, but in **Intel Hex** (`iHex`) format.


**Flashing Instructions**

- To write **User bitstream** without **User**/**Golden** Support:
  `python -m boards.targets.limesdr_mini_v2 --flash`
- To write the **User bitstream** (assuming it's already built):
  `python -m boards.targets.limesdr_mini_v2 --flash-user`
- to write **Golden bitstream**:
  `python -m boards.targets.limesdr_mini_v2 --flash-golden`
- To write the **full Flash image**:
  `openFPGALoader --c CABLE limesdr_mini_v2.mcs`

### limesdr_xtrx

```bash
# Build the Gateware
python -m boards.targets.limesdr_xtrx --build [--with-bios] [--load] [--write]
```

Where:

- `--with-bios` enables *LiteX bios*
- `--load` to load the bitstream (Volatile memory)
- `--write` to write the bitstream (SPI Flash)


## LimeSDR Mini v1 Boot

This section explains why two distinct bitstreams are required:
- one for the **Golden** bitstream
- one for the **Operational** bitstream.

### MAX10 Limitations in Dual Image Configuration Mode

The Intel MAX10 FPGA has certain restrictions in *Dual Image Configuration Mode*:
- **RAM cannot be used as ROM**, leading to increased resource usage. This limits
  the available features in the gateware, which is why the LMS7002 transceiver and
  RX/TX paths are disabled in the Golden bitstream.
- **Preinitialized ROM can only be configured once**. If the operational bitstream
  relies on ROM, the Golden bitstream cannot.

The solution to bypass these limitations is:
- using *UFM* section (internal flash) to store firmware for operational: no ROM are
  used allowing a full featured gateware
- using *ROM* only for Golden: this bitstream will be able to erases/reprograms internal flash
  but has nothing related to the RF

Operational bitstream can't be used to reprogram internal_flash because firmware is
executed directly from the Flash, but this one will be erased during the update sequence.

### Solution

To work around these limitations:
- The **UFM (User Flash Memory) section** is used to store firmware for the operational
  bitstream. Since no ROM is required, this allows for a fully featured gateware and
  let **Golden bitstream** using ROM.
- **ROM is used only for the Golden bitstream**, which is responsible for erasing
  and reprogramming the internal flash but does not interact with RF functionality.

### Why the Operational Bitstream Cannot Reprogram Internal Flash

The operational bitstream cannot be used for internal flash reprogramming because
its firmware runs directly from flash memory. During an update, this memory is
erased, making it unavailable for execution.

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
