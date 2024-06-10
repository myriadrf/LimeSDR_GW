# LimeSDR-XTRX LiteX Gateware infrastructure

The objective of this project is to provide the necessary infrastructure to enable Lime Microsystems
to efficiently develop and maintain LiteX-based gateware for their LimeSDR XTRX product.

This repository is based on the [LimeSDR-XTRX LiteX Gateware](https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW)
repository with the following added/tested features:
- Board programming/tools (SRAM and SPI Flash).
- VexRiscv SoftCore CPU with debug capabilities over JTAG and interrupts.
- PCIe core with MMAP and DMA interfaces (AXI-MMAP and AXI-ST).
- CSR registers example.
- VHDL integration example.
- LiteScope example over JTAG or PCIe.
- Firmware loading from UART.
- Firmware loading from SPI Flash.
- Firmware debug through GDB over JTAG.
- Interrupt handling example.

## Cloning the Repository

To clone this repository and initialize the submodules, run the following commands:

```bash
git clone https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW.git
git submodule init 
git submodule update
```

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

## Build Gateware and Load Firmware through JTAG

```bash
# Build and load gateware bitstream:
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc --with-privileged-debug --hardware-breakpoints 4

# Build firmware:
cd firmware && make clean all && cd ../

# Load CPU firmware:
litex_term jtag --jtag-config=openocd_xc7_ft2232.cfg --kernel firmware/demo.bin
```
### Load Firmware through PCIe

First, load the *litepcie* driver:

TBD

```bash
litex_term /dev/ttyLXU0 --kernel firmware/demo.bin
```

## Load Gateware trough JTAG (volatile and non-volatile memory)

[openFPGALoader](https://github.com/trabucayre/openFPGALoader) is used to load
and/or to flash bitstream.

By default, a **digilent_hs2** *USB-JTAG* cable will be used. To change this
behaviour and to select an alternate cable, one must appends  command line with
`--cable xxx` where `xxx` is the cable's name (see `openFPGALoader --list-cables`
for a complete list of supported cbles).

After bitstream loaded/flashed, computer must be rebooted or a PCIe Bus rescan
must be performed:
```bash
# Get PCIe location
lspci | grep grep -i RF_controller
# remove device (replace X with actual value, see previous command)
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0X\:00.0/remove
# before rescan flash/load new bitstream
echo 1 | sudo tee /sys/bus/pci/rescan
```

## RAM (volatile memory)

**Note:** in this mode gateware will be lost after power cycle.

```bash
python3 limesdr_xtrx.py --load [--cable XXX]
```

## SPI Flash (non-volatile memory)

**Note:** in this mode gateware will be automatically loaded after flash and after power cycles.

```bash
python3 limesdr_xtrx.py --flash [--cable XXX]
```

## Writing firmware in SPI flash (instead of shipped into gateware)

By default the CPU's firmware is included into the gateware but its also possible to have an
external firmware written to the SPI flash. The benefits is to be able to update firmware without
having to rebuild a full gateware. To enable this option user must use command similar to:
```bash
python limesdr_xtrx.py --build --flash-boot --flash [--bios-flash-offset 0xXXXXX]
```

By default CPU's firmware will be written at 0x220000, with `--with-flash-offset` user is
free to provides a different offset.

## Using GpioTop (connected to led2)

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

## Firmware debug

A gateware with debug interface must be build and loaded:

```bash
./limesdr_xtrx.py --with bscan --build --load --flash


# Load firmware trough serial
litex_term /dev/ttyLXU0 --kernel firmware/demo.bin

# Run OpenOCD with the specified configurations:
openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connecting GDB for Debugging:
gdb-multiarch -q firmware/demo.elf -ex "target extended-remote localhost:3333"
```

Note that instead of usign GDB directly, Eclipse IDE can be configured to debug code in more user friendly way. Follow this guide to configure Eclipse IDE:

https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software