# LimeSDR-XTRX Gateware 

Cloning repo:
```
git clone https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW.git
git submodule init 
git submodule update
```

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

## Load Gateware trough JTAG (volatile and non-volatile memory)

* [openFPGALoader](https://github.com/trabucayre/openFPGALoader) is used to load and/or to flash bitstream.

* By default, a **digilent_hs2** *USB-JTAG* cable will be used. To change this behaviour and to select an
  alternate cable, one must appends  command line with `--cable xxx` where `xxx` is the cable's name (see `openFPGALoader --list-cables` for a complete list of supported cbles).

After bitstream loaded/flashed, computer must be rebooted or a PCIe Bus rescan must be performed:
```bash
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0X\:00.0/remove (replace X with actual value)
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

