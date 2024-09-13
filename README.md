# LimeSDR Gateware

This project focuses on the development and customization of FPGA gateware for LimeSDR boards using the LiteX framework. 

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

## Requirements

In order to build broject these tools/software are required:
  - **Litex** - follow offcial install instructions in [repository](https://github.com/enjoy-digital/litex) 
  - **Vivado 2022.1** - free version can be downloaded from [www.xilinx.com](https://www.xilinx.com/support/download.html)
  
## Building and loading the Gateware

To build the gateware, use the following command with the appropriate board option:

```bash
./limesdr_xtrx.py --build --board=limesdr [--cable ft2232] [--load] [--flash]
```

**Available boards for `--board` option are:**
- limesdr

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

## Building/loading VexRiscv firmware trough UART

By default firmware should be built when building gateware and compiled into SRAM. Separately firmware can by compiled and loaded trough UART:

```bash
# Build firmware:
cd firmware && make clean all && cd ../

# Load firmware trough serial
litex_term  /dev/ttyUSB0 --kernel firmware/firmware.bin --csr-csv csr.csv
```

## Documentation

More details can be found in:
* https://limesdrgw.myriadrf.org/