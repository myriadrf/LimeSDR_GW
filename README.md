# LimeSDR Gateware

## Available branches

  - **master** - most recent stable and tested work 
  - **develop** - ongoing development

## Cloning the Repository

To clone this repository and initialize the submodules, run the following commands:

```bash
git clone https://github.com/myriadrf/LimeSDR_GW.git
git submodule init 
git submodule update
```

## Building the Gateware

To build the gateware, use the following command with the appropriate board option:

```bash
./limesdr_xtrx.py --build --board=limesdr [--cable ft2232] [--load] [--flash]
```

**Available boards for `--board` option are:**
- limesdr

The `--load` and `--flash` options update the FPGA and/or Flash as described below.

**Additional options include:**

- `--cable` to specify JTAG cable to be used.
- `--with-bscan` to add JTAG access to the *vexriscv-smp* softcore for debugging.
- `--load` load bitstream to SRAM
- `--flash` load bitstream to FLASH memory
- `--flash-boot` to flash the CPU Softcore's firmware in SPI (requires `--flash`).

**Note:** `--load`, `--flash`, and `--flash-boot` use **JTAG** to communicate with the FPGA. These
  actions require an external probe, with a *digilent_hs2* cable used by default. Use `--cable`
  followed by the cable name to change this (see `openFPGALoader --list-cables` for supported
  cables).

## Gateware loading/flashing to SRAM and SPI Flash.

[openFPGALoader](https://github.com/trabucayre/openFPGALoader) is used to load and/or flash
bitstreams and CPU firmware.

By default, a **digilent_hs2** *USB-JTAG* cable will be used. To change this behavior and select an
alternate cable, append the command line with `--cable xxx`, where `xxx` is the cable's name
(see `openFPGALoader --list-cables` for a complete list of supported cables).

### Loading Gateware to SRAM (Volatile Memory)

In this mode, the gateware will be lost after a power cycle.

```bash python3 limesdr_xtrx.py --load [--cable XXX] ```

### Flashing Gateware to SPI Flash (Non-Volatile Memory)

In this mode, the gateware will be automatically loaded after flashing and power cycles.

```bash python3 limesdr_xtrx.py --flash [--cable XXX] ```

## Building/loading VexRiscv firmware trough UART

By default firmware should be built when building gateware and compiled into SRAM. Separately firmware can by compiled and loaded trough UART:

```bash
# Build firmware:
cd firmware && make clean all && cd ../

# Load firmware trough serial
litex_term  /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv
```

## VexRiscv-SMP SoftCore CPU with Debug Capabilities

VexRiscv-SMP SoftCore has been integrated and configured in the `limesdr_xtrx.py` target design. The
configuration has IRQ enabled (which have been recently added to upstream LiteX) and debug
capabilities over JTAG, which were the two requirements for the CPU. VexRiscv-SMP IRQs are enabled
by default. To enable debug capabilities in the target, the additional command lines are added:

```python
if with_bscan:
    from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP
    VexRiscvSMP.privileged_debug     = True
    VexRiscvSMP.hardware_breakpoints = 4
    VexRiscvSMP.with_rvc             = True
```


## Firmware Debug through GDB over JTAG

To build and load a gateware with a debug interface:

```bash
./limesdr_xtrx.py --with bscan --build --load --flash

# Load firmware through serial:
litex_term /dev/ttyLXU0 --kernel firmware/firmware.bin

# Run OpenOCD with the specified configurations:
openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connect GDB for debugging:
gdb-multiarch -q firmware/firmware.elf -ex "target extended-remote localhost:3333"
```

Note that instead of using GDB directly, Eclipse IDE can be configured to debug code in a more
user-friendly way. Follow this guide to configure Eclipse IDE:
[Using Eclipse to run and debug the software](https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software)

### JTAGBone Test

First step is to start a server configured for JTAG access:
```bash
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```

`--jtag-config` must be adapted according to the JTAG interface

Now it's possible to read/write `gpioTop` registers:

Value read:
```bash
litex_cli --read lime_top_gpio_gpio_val
```

HOST access (direction/level)
```bash
# control overriden for all pins
litex_cli --write lime_top_gpio_gpio_override 0x07
# set all pins as output
litex_cli --write lime_top_gpio_gpio_override_dir 0x00
# pins 0 & 2 low, pin 1 high
litex_cli --write lime_top_gpio_gpio_override_val 0x05
```