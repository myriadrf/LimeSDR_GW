# LimeSDR-XTRX Gateware 

## Build gateware and load firmware trough JTAG
Build/Load gateware bitstream:
```
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4
```

Build firmware:
```
cd firmware && make clean all && cd ../
```

Load CPU firmware:
```
litex_term jtag --jtag-config=openocd_xc7_ft2232.cfg --kernel firmware/demo.bin
```

## Debug CPU code trough JTAG and GDB
Build/Load gateware bitstream:
```
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4
```

Build firmware:
```
cd firmware && make clean all && cd ../
```
Run OpenOCD with the specified configurations:
```
openocd -f ./prog/openocd_xc7_ft2232.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl
```
Connecting GDB for Debugging:
```
gdb-multiarch -q firmware/demo.elf -ex "target extended-remote localhost:3333"
```




