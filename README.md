# LimeSDR-XTRX Gateware 

Cloning repo:
```
git clone https://github.com/vbuitvydas/LimeSDR-XTRX_LiteX_GW.git
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




