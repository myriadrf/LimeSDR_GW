# LimeSDR-XTRX LiteX Gateware infrastructure

The objective of this project is to provide the necessary infrastructure to enable Lime Microsystems
to efficiently develop and maintain LiteX-based gateware for their LimeSDR XTRX product.

This repository is based on the [LimeSDR-XTRX LiteX Gateware](https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW)
repository with the following added/tested features:
- VexRiscv SoftCore CPU with debug capabilities over JTAG and interrupts.
- PCIe core with MMAP and DMA interfaces (AXI-MMAP and AXI-ST).
- FPGA programming to SRAM and SPI Flash.
- Firmware loading from PCIe.
- Firmware loading from SPI Flash.
- Firmware debug through GDB over JTAG.
- Firmware interrupt handling example.
- CSR registers integration example.
- VHDL integration example.
- LiteScope logic analyzer example


## Cloning the Repository

To clone this repository and initialize the submodules, run the following commands:

```bash
git clone https://github.com/enjoy-digital/LimeSDR-XTRX_LiteX_GW.git
git checkout litex_infrastructure_setup
git submodule init 
git submodule update
```

## Building the Gateware

Basic usage to build the gateware:

```bash
./limesdr_xtrx.py --build [--load] [--flash]
```

This command builds the gateware for a *fairwaves_pro* XTRX board. To select another variant, use
the `--board` option:

- `fairwaves_cs` for the commercial version based on *35t*.
- `limesdr` for the *MyriadRF* latest variant.

The `--load` and `--flash` options update the FPGA and/or Flash (details below).

Additional options include:

- `--with-bscan` to add JTAG access to the *vexriscv-smp* softcore for debugging.
- `--flash-boot` to flash the softcore's firmware in SPI (requires `--flash`).

**Note:** `--load`, `--flash`, and `--flash-boot` use **JTAG** to communicate with the FPGA. These
  actions require an external probe, with a *digilent_hs2* cable used by default. Use `--cable`
  followed by the cable name to change this (see `openFPGALoader --list-cables` for supported
  cables).

## VexRiscv SoftCore CPU with Debug Capabilities

TODO

## PCIe Core with MMAP and DMA Interfaces

TODO

# FPGA Programming to SRAM and SPI Flash

### Load Gateware through JTAG (volatile and non-volatile memory)

[openFPGALoader](https://github.com/trabucayre/openFPGALoader) is used to load and/or to flash bitstream.

By default, a **digilent_hs2** *USB-JTAG* cable will be used. To change this behaviour and to select an alternate cable, one must append the command line with `--cable xxx` where `xxx` is the cable's name (see `openFPGALoader --list-cables` for a complete list of supported cables).

After bitstream loaded/flashed, computer must be rebooted or a PCIe Bus rescan must be performed:

```bash
# Get PCIe location
lspci | grep -i RF_controller
# Remove device (replace X with actual value, see previous command)
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0X\:00.0/remove
# Rescan PCIe Bus after flashing/loading new bitstream
echo 1 | sudo tee /sys/bus/pci/rescan
```

### RAM (volatile memory)

**Note:** In this mode, the gateware will be lost after a power cycle.

```bash
python3 limesdr_xtrx.py --load [--cable XXX]
```

### SPI Flash (non-volatile memory)

**Note:** In this mode, the gateware will be automatically loaded after flashing and power cycles.

```bash
python3 limesdr_xtrx.py --flash [--cable XXX]
```

### Writing Firmware in SPI Flash (Instead of including it in Gateware)

By default, the CPU's firmware is included in the gateware, but it is also possible to have an external firmware written to the SPI flash. This allows firmware updates without rebuilding the full gateware. To enable this option, use a command similar to:

```bash
python limesdr_xtrx.py --build --flash-boot --flash [--bios-flash-offset 0xXXXXX]
```

The default offset for the CPU's firmware is 0x220000. Use `--with-flash-offset` to specify a different offset.

# Firmware Loading from PCIe

First, load the *litepcie* driver:

```bash
cd software/kernel
make clean all
sudo ./init.sh
```

Then load the firmware over PCIe:

```bash
litex_term /dev/ttyLXU0 --kernel firmware/demo.bin
```

## Firmware Loading from SPI Flash

TODO

## Firmware Debug through GDB over JTAG

To build and load a gateware with a debug interface:

```bash
./limesdr_xtrx.py --with bscan --build --load --flash

# Load firmware through serial:
litex_term /dev/ttyLXU0 --kernel firmware/demo.bin

# Run OpenOCD with the specified configurations:
openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connect GDB for debugging:
gdb-multiarch -q firmware/demo.elf -ex "target extended-remote localhost:3333"
```

Note that instead of using GDB directly, Eclipse IDE can be configured to debug code in a more user-friendly way. Follow this guide to configure Eclipse IDE:

[Using Eclipse to run and debug the software](https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software)


## Firmware Interrupt Handling Example

TODO

## CSR Registers Integration Example

TODO

## VHDL Integration Example

Using GpioTop (connected to led2)

### Target update

To uses `user_led2` with `GpioTop` instead of `LedChaser`:
```python
#self.leds2 = LedChaser(
#    pads         = platform.request_all("user_led2"),
#    sys_clk_freq = sys_clk_freq
#)
from gateware.GpioTop import GpioTop
self.gpio = GpioTop(platform, platform.request_all"user_led2"))
# Set all gpio to outputs
self.comb += self.gpio.GPIO_DIR.eq(0b000)
self.comb += self.gpio.GPIO_OUT_VAL.eq(0b010)
```

**Note:** when `GPIO_DIR` is set as input, bitstream fails due to `IOBUF.O`
unrouted.

### JTAGBone access

First step is to start a server configured for JTAG access:
```bash
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```

`--jtag-config` must be adapted according to the JTAG interface

Now it's possible to read/write `gpioTop` registers:

Value read:
```bash
litex_cli --read gpio_gpio_val
```

HOST access (direction/level)
```bash
# control overriden for all pins
litex_cli --write gpio_gpio_override 0x07
# set all pins as output
litex_cli --write gpio_gpio_override_dir 0x00
# pins 0 & 2 low, pin 1 high
litex_cli --write gpio_gpio_override_val 0x05
```

## LiteScope Logic Analyzer Example

TODO
