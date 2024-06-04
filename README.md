# LimeSDR-XTRX Gateware 

Cloning repo:
```
git clone https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW.git
git submodule init 
git submodule update
```

## Build gateware

Basic usage:
```bash
./limesdr_xtrx.py --build [--load] [--flash]
```

This command will build the gateware for a *fairwaves_pro* XTRX board.
To select another variant user needs to use `--board` with:
- `fairwaves_cs` for first version based on *35t*
- `limesdr` for the *MyriadRF* lastest (current) variant

`--load` and/or `--flash` will update FPGA and/or Flash (see below for details).

Target script provides some aditional options (details are provided in further sections):
- `--with-bscan` to add JTAG access to the *vexriscv-smp* softcore for debug purpose
- `--flash-boot` to flash softcore's firmware in SPI (must be used with `--flash` option)

*Note:* `--load`, `--flash` and `--flash-boot` uses **JTAG** to communicates with the FPGA, these actions requires
an external probe. By default, a *digilent_hs2* cable will be used, to change this behavior user
must uses `--cable` followed by the cable name (see `openFPGALoader --list-cables` to see full list of
supported cables.)

## Build gateware, load firmware trough GPIO UART and launch CPU debug trough JTAG

Required hardware:
1. LimeSDR-XTRX v1.2 board
2. Mini PCIe to PCIe adapter
3. FT2232H mini module connected to PCIe adapter JTAG:
    ```
    JTAG       FT2232H
    ------------------
    TMS   ->   CN2_12 
    TDI   ->   CN2_10
    TDO   ->   CN2_9
    TCK   ->   CN2_7
    GND   ->   CN2_2
    VIO   ->   CN2_11
    ``` 
4. USB Serial converter connected to PCIe adapter GPIO
   ```
   GPIO              Serial 
   ------------------------
   GPIO3N(RX)   <-   TX
   GPIO3P(TX)  ->    RX
   ```

Steps to load build/load gateware, load firmware and launch debug:

```
# Build/Load gateware bitstream:
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=gpio_serial --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4 --csr-csv=csr.csv

# Build firmware:
cd firmware && make clean all && cd ../

# Load firmware trough serial
litex_term --csr-csv csr.csv /dev/ttyUSB0 --kernel firmware/demo.bin

# Run OpenOCD with the specified configurations:
openocd -f ./limesdr_xtrx.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connecting GDB for Debugging:
gdb-multiarch -q firmware/demo.elf -ex "target extended-remote localhost:3333"
```

Note that instead of usign GDB directly, Eclipse IDE can be configured to debug code in more user friendly way. Follow this guide to configure Eclipse IDE:

https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software


## Build gateware and load firmware trough JTAG

```
# Build/Load gateware bitstream:
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4

# Build firmware:
cd firmware && make clean all && cd ../

# Load CPU firmware:
litex_term jtag --jtag-config=openocd_xc7_ft2232.cfg --kernel firmware/demo.bin
```

## Load firmware through PCIe

First *litepcie* driver must be loaded (TBD).
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
python3 litex_xtrx --load [--cable XXX]
```

## SPI Flash (non-volatile memory)

**Note:** in this mode gateware will be automatically loaded after flash and after power cycles.

```bash
python3 litex_xtrx --flash [--cable XXX]
```

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

```
./litex_xtrx.py --with bscan --build --load --flash


# Load firmware trough serial
litex_term /dev/ttyLXU0 --kernel firmware/demo.bin

# Run OpenOCD with the specified configurations:
openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connecting GDB for Debugging:
gdb-multiarch -q firmware/demo.elf -ex "target extended-remote localhost:3333"
```
