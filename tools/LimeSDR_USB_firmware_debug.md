# Interactive Firmware Debugging (GDB over JTAG) — LimeSDR-USB

This document describes how to interactively debug the LimeSDR-USB firmware (running on the
VexRiscv softcore in the Cyclone IV `EP4CE40`) with GDB, over the board's existing FT2232 JTAG
cable, using **only upstream, stock tooling**. It is an **opt-in** feature: it must be explicitly
enabled at build time via `--with-cpu-debug` and has no effect on production builds.

> [!IMPORTANT]
> Like the rest of `tools/`, this workflow is intended for **internal development use**.

> [!NOTE]
> The LimeSDR-USB target exposes **two independent, opt-in JTAG facilities**, and they are
> **mutually exclusive** because both consume the board's single Altera Virtual-JTAG instance:
>
> - `--with-cpu-debug` — **firmware source-level debugging** (GDB: breakpoints, single-step,
>   registers/memory). This is what the bulk of this document covers.
> - `--with-jtagbone` — **gateware debugging / register monitoring**: a wishbone-over-JTAG bus
>   master driven from the host with `litex_server` + `litex_cli` (CSR peek/poke, LiteScope).
>   See *Gateware debugging / register monitoring (JTAGBone)* below.
>
> Passing both flags together is rejected at build time.

## How it works

`--with-cpu-debug` swaps the production `vexriscv` `minimal` core for a **single-core
`vexriscv_smp`** configured with the official RISC-V debug spec (`privileged_debug`), compressed
instructions (`with_rvc`), and 4 hardware breakpoint comparators. This exposes a **spec-compliant
RISC-V Debug Module** whose debug port is tunneled out through the board's single **Altera
Virtual-JTAG** instance (`sld_virtual_jtag`, wired in
`boards/targets/limesdr_usb.py::add_jtag_cpu_debug()`).

Debugging then uses the **same framework as the XTRX/SSDR/HiperSDR reference boards**: stock
upstream OpenOCD driving the FT2232 cable directly, plus the repo's shared
`riscv_jtag_tunneled.tcl` tunnel script.

```
riscv32-unknown-elf-gdb  ->  OpenOCD (upstream + riscv_jtag_tunneled.tcl)  ->  FT2232 JTAG
                         ->  Altera Virtual-JTAG tunnel  ->  vexriscv_smp RISC-V Debug Module
```

There is **no** `litex_server`, **no** etherbone bridge, **no** forked OpenOCD, and **no**
`litex_server` patch step. This 2-hop path replaces the previous fragile multi-process chain (see
*Recovering the legacy stack* at the end).

Because the debug DM is a native RISC-V target, GDB `break`/`watch` can use the CPU's **4 hardware
trigger comparators**, which fire directly against the **read-only ROM**-resident firmware without a
`--with-bios` rebuild. This is not automatic, though: GDB defaults to *software* breakpoints (it
inserts an `ebreak` by writing to the code), and since the firmware lives in ROM that write is
silently ignored — the breakpoint never fires, and source-level `step`/`next` (which GDB implements
with temporary breakpoints) then run forever without ever stopping. `tools/limesdr_usb_debug.cfg`
therefore sets `gdb_breakpoint_override hard`, so OpenOCD turns every GDB breakpoint request into a
hardware one. Do **not** remove that line — without it only `halt`/`interrupt` works, while
breakpoints and stepping silently fail.

> [!NOTE]
> `--with-cpu-debug` implies `--no-ddr`: the heavier `vexriscv_smp` core trades the DDR modules
> for FPGA resource headroom on the small `EP4CE40`. The debug bitstream is therefore for
> firmware bring-up/debugging, not for exercising the DDR data path.

## Prerequisites

- **Upstream OpenOCD with RISC-V support** (0.12+ / a recent `riscv-openocd`). No fork is
  required — the stock RISC-V tunnel commands (`riscv use_bscan_tunnel`,
  `riscv set_bscan_tunnel_ir`) are all that is used. Many distro packages are too old; build a
  recent OpenOCD if `riscv use_bscan_tunnel` is unknown.
- A RISC-V GDB, e.g. `riscv32-unknown-elf-gdb` (the same toolchain family used to build the
  firmware, see `docs/docs/firmware_toolchains.rst`).
- The board connected via the FT2232 JTAG cable (the same cable used for `--load`/`--flash`).

## Workflow

1. **Build with CPU debug enabled** (this also implies `--no-ddr`):

   ```bash
   python3 boards/targets/limesdr_usb.py --build --with-cpu-debug
   ```

2. **Load the resulting bitstream** onto the board (e.g. with `--load`, or by flashing).

   ```bash
   python3 boards/targets/limesdr_usb.py --load
   ```

3. **Start OpenOCD** from the repository root, giving it the board interface config and the shared
   tunnel target script:

   ```bash
   openocd -f tools/limesdr_usb_debug.cfg -f riscv_jtag_tunneled.tcl
   ```

   OpenOCD opens the FT2232 cable, selects the Altera Virtual-JTAG tunnel, halts the CPU, and
   opens a standard GDB remote-serial port (default `3333`), printing `Listening on port 3333 for
   gdb connections`. Leave it running in its own terminal.

4. **Attach GDB** to the firmware ELF:

   ```bash
   riscv32-unknown-elf-gdb firmware/firmware.elf
   (gdb) set remotetimeout 20
   (gdb) target remote :3333
   ```

   From here, standard GDB commands work as expected: `break`, `continue`, `step`, `next`,
   `stepi`, `watch`, `bt`, and register/memory read/write. Breakpoints and watchpoints are backed
   by the core's **4 hardware comparators**, so they fire directly against the ROM-resident
   firmware. An interrupt (Ctrl-C) followed by `continue` resumes cleanly without tearing down the
   session.

   > [!NOTE]
   > There are **4** hardware breakpoints (`VexRiscvSMP.hardware_breakpoints = 4` in
   > `boards/targets/limesdr_usb.py`). A 5th `break`/`watch` will fail to insert — delete one
   > first. If the debug bitstream ever runs short on `EP4CE40` resources, this count (and the
   > core's caches/features) is the first thing to trim.

5. **Validate the workflow end-to-end** (optional). Given an already-built/programmed debug board,
   this launches OpenOCD, drives a scripted GDB batch, and asserts the three behaviours the old
   stack could not do reliably — a `break main` that actually fires, a single-step that advances
   the PC, and an interrupt+`continue` cycle that does not drop the session — printing a single
   PASS/FAIL verdict:

   ```bash
   tools/validate_usb_cpu_debug.sh
   ```

   Run `tools/validate_usb_cpu_debug.sh --help` for options (custom ELF/GDB binary, port).

## On-hardware validation checklist

Run through these once on real hardware after any change to the gateware debug wiring, the host
config, or the OpenOCD/GDB toolchain:

- [ ] `openocd -f tools/limesdr_usb_debug.cfg -f riscv_jtag_tunneled.tcl` reaches `halt` and prints
      `Listening on port 3333 for gdb connections` (no `litex_server`, fork, or patch involved).
- [ ] GDB attaches with `target remote :3333` and `info registers` returns sane values.
- [ ] `break main` + `continue` **fires** (hardware breakpoint against ROM).
- [ ] `stepi`/`step`/`next` advance the PC.
- [ ] Memory read/write works (`x/8xw 0x0`, write a scratch SRAM word and read it back).
- [ ] Ctrl-C (interrupt) then `continue` repeatedly does **not** tear the session down.
- [ ] Inserting a 5th hardware breakpoint fails cleanly (4-comparator limit).

## Troubleshooting

| Symptom | What to check / do |
|---|---|
| `unable to open ftdi device` / `LIBUSB_ERROR_BUSY` | The FT2232 is claimed by another process (a running `litex_server`, a previous OpenOCD, or the Linux `ftdi_sio` driver). Close the other user; the cable is shared with `--load`/`--flash`. |
| `Error ... does not have valid IDCODE` / IDCODE mismatch | The device TAP settings in `tools/limesdr_usb_debug.cfg` (`-irlen 10 -expected-id 0x020F40DD`) do not match the connected device. Confirm the board is an `EP4CE40F23C8` and re-check the IDCODE against the Cyclone IV E device handbook. |
| Device IDCODE reads OK (`0x020f40dd`) but then `expected 1 of 1: 0x10003fff`, `IR capture error; saw 0x3f not 0x01`, `dtmcontrol is 0`, `examination failed` | The TAP is being created with the wrong IR length. This happens if the device TAP is named `riscv.cpu`: `riscv_jtag_tunneled.tcl` then treats it as its **own** target and overrides it with a Xilinx-style irlen-6 / `0x10003FFF` TAP. `tools/limesdr_usb_debug.cfg` avoids this by naming the TAP `ep4ce40.tap` (distinct from the script's `riscv.cpu` target) — never rename it back to `riscv.cpu`. See *Diagnosing a dead tunnel* below. |
| Tunnel selected but the DM is never examined | The Altera virtual-JTAG tunnel instruction may differ from the assumed `USER1`. `tools/limesdr_usb_debug.cfg` sets `BSCAN_TUNNEL_IR 0x00e`; this is the value `riscv_jtag_tunneled.tcl` feeds to `riscv set_bscan_tunnel_ir` (look for `Bscan tunnel IR 0xe selected` in the OpenOCD log). Re-check the Cyclone IV `USER1` opcode if the DM does not respond. |
| `riscv: unknown command` in OpenOCD | Your OpenOCD is too old / built without RISC-V support. Build a recent upstream OpenOCD (see Prerequisites). |
| `break` never fires, or `step`/`next` never stops and the CPU just keeps running | The firmware is in read-only ROM, so GDB's default *software* breakpoints (and the temporary breakpoints source-stepping relies on) are written to ROM and silently ignored. `tools/limesdr_usb_debug.cfg` must contain `gdb_breakpoint_override hard` so OpenOCD converts them to hardware breakpoints — confirm it is present (OpenOCD prints `force hard breakpoints` at startup). Only `halt`/Ctrl-C works without it. |
| `break`/`watch` fails to insert once several are set | You exceeded the **4** hardware comparators (`info breakpoints`; `delete` one). With `gdb_breakpoint_override hard` every breakpoint is a hardware one, so all four are shared between `break` and `watch`. |

### Diagnosing a dead tunnel

The physical device TAP (IDCODE, IR length) and the tunneled RISC-V Debug Module are **two
separate layers**. When the OpenOCD log shows the correct device IDCODE (`0x020f40dd`) but then
`dtmcontrol is 0` / `Target not examined yet`, the physical cable is fine and the problem is in
the TAP definition or the tunnel. A healthy run instead looks like:

```
Info : JTAG tap: ep4ce40.tap tap/device found: 0x020f40dd ...
Info : [riscv.cpu.0] Examined RISC-V core; found 1 harts
Info : [riscv.cpu.0]  XLEN=32, misa=0x40141105
[riscv.cpu.0] Target successfully examined.
```

Work through the following on hardware, in order:

1. **Check the TAP name and IR length first (most common cause).** In the log, the line
   `JTAG tap: <name> tap/device found: 0x020f40dd` must show the TAP named **`ep4ce40.tap`** with a
   **10-bit** IR. If you instead see `expected ... 0x10003fff` and `IR capture error; saw 0x3f not
   0x01`, the shared `riscv_jtag_tunneled.tcl` has replaced the real EP4CE40 TAP with its own
   Xilinx-style irlen-6 TAP. That happens whenever the device TAP is named `riscv.cpu` (identical to
   the script's own target name). Keep the `set _CHIPNAME ep4ce40` / `set TAP_NAME ep4ce40.tap`
   naming in `tools/limesdr_usb_debug.cfg` — do not rename the TAP to `riscv.cpu`.

2. **Confirm the tunnel IR is being selected.** The log must contain `Simple Register based Bscan
   Tunnel Selected` and `Bscan tunnel IR 0xe selected`. If not, `BSCAN_TUNNEL_IR` is not reaching
   `riscv set_bscan_tunnel_ir`.

3. **If the TAP is correct but the DM still does not examine, capture a full debug trace** and read
   the tunneled DTMCS scan directly, then share the log:

   ```bash
   openocd -f tools/limesdr_usb_debug.cfg -f riscv_jtag_tunneled.tcl \
           -c "init" -c "riscv dmi_read 0x10" -c "riscv dmi_read 0x11" -c "shutdown" -d3 \
           2>&1 | tee /tmp/usb_cpu_debug_openocd.log
   ```

   In the trace, find the `dtmcontrol_scan` / `DTMCS` line. `DTMCS -> 0xffffffff` (or `-> 0x0`)
   means the nested tunnel scan is misaligned; a sane value (non-zero `version` field) means the
   tunnel works and the problem is downstream (reset/clock — see below). Remaining candidates, in
   order of likelihood:
   - **Wrong `USER1` opcode.** `0x00e` is the standard Cyclone IV E `USER1` (LiteX itself uses
     `0xe` for `ep4ce` JTAG streaming); if needed, try `set BSCAN_TUNNEL_IR 0x00c` (`USER0`).
   - **Tunnel-IR width mismatch.** `riscv_jtag_tunneled.tcl` uses `riscv use_bscan_tunnel 6 1`
     (6-bit nested IR, data-register tunnel type), matching VexRiscv's `EmbeddedRiscvJtag`. Do not
     change this unless the CPU's debug-transport IR width was customised.
   - **Adapter clock too high for the tunnel.** The script forces `adapter speed 500` (500 kHz);
     tunneled scans are far more timing-sensitive than plain configuration.

4. **If DTMCS is sane but the target still drops (connects, then loses the DM)**, the debug
   module is sharing a reset with the CPU. The `cd_jtag` domain in
   `boards/targets/limesdr_usb.py::add_jtag_cpu_debug()` is intentionally kept off the CPU reset;
   verify no later edit re-coupled them.

## Interface / target files

- `tools/limesdr_usb_debug.cfg` — OpenOCD interface + TAP config: the on-board FT2232
  (`adapter driver ftdi`), the `EP4CE40` device TAP (`-irlen 10 -expected-id 0x020F40DD`, named
  `ep4ce40.tap` — deliberately **not** `riscv.cpu`, see *Diagnosing a dead tunnel*), the
  `TAP_NAME` binding so the shared tunnel script attaches its RISC-V target to this TAP, and
  `BSCAN_TUNNEL_IR 0x00e` selecting the Altera `USER1` tunnel instruction.
- `riscv_jtag_tunneled.tcl` (repo root) — the **shared** RISC-V BSCAN-tunnel target script, reused
  as-is by XTRX/SSDR/HiperSDR. It gained one backward-compatible hook: when a board's interface
  config sets `BSCAN_TUNNEL_IR`, it calls `riscv set_bscan_tunnel_ir` for non-Xilinx (Altera)
  tunnels. Xilinx boards leave it unset and are unaffected.
- `tools/validate_usb_cpu_debug.sh` — optional end-to-end validator (OpenOCD + a scripted GDB
  batch) printing a single PASS/FAIL verdict.
- `tools/limesdr_usb_jtagbone.cfg` — OpenOCD interface + TAP config for the **`--with-jtagbone`**
  gateware/register-monitoring flow (`litex_server --jtag`), separate from the firmware GDB path.

## Gateware debugging / register monitoring (JTAGBone)

When the goal is **not** firmware source-level debugging but inspecting the *gateware* — reading and
writing CSRs/registers, poking the wishbone bus, or attaching a LiteScope analyzer — build with
`--with-jtagbone` instead of `--with-cpu-debug`:

```bash
python3 boards/targets/limesdr_usb.py --build --with-jtagbone
```

Unlike `--with-cpu-debug`, this keeps the **production `vexriscv` `minimal` core unchanged** and
does **not** imply `--no-ddr`; it merely adds a `JTAGBone` wishbone bus master reached over the
board's Altera Virtual-JTAG (the same FT2232 cable used for `--load`/`--flash`).

After loading the bitstream, bridge the JTAG bus to a local server and use the standard LiteX host
tools:

```bash
# Terminal 1: expose the SoC wishbone/CSR space over JTAG.
litex_server --jtag --jtag-config tools/limesdr_usb_jtagbone.cfg

# Terminal 2: read/write CSRs by name (from the generated csr.csv).
litex_cli --regs                 # dump all CSRs
litex_cli --read  <csr_name>
litex_cli --write <csr_name> <value>
```

`litex_server --jtag` needs an OpenOCD interface config describing the FT2232 adapter and the
device TAP. `tools/limesdr_usb_jtagbone.cfg` provides exactly that (the on-board FT2232 plus the
`EP4CE40` TAP); the legacy `tools/limesdr_usb.cfg` that served the same purpose is also recoverable
from the git stash described below.

> [!IMPORTANT]
> `--with-jtagbone` and `--with-cpu-debug` cannot be combined — both need the single
> `sld_virtual_jtag` instance on this Cyclone IV, so the target raises an error if both are
> requested. Pick the facility that matches the task: JTAGBone for gateware/register work, CPU
> debug for firmware GDB.

## Recovering the legacy stack

The previous debug implementation — `GDB -> OpenOCD (SpinalHDL fork) -> litex_server (patched) ->
FT2232 -> jtagbone -> VexRiscv wishbone debug bus` — was removed in favour of the native path
above. Its host-side files (`tools/limesdr_usb.cfg`, `tools/openocd_vexriscv_debug.cfg`,
`tools/limesdr_usb_debug.gdb`, `tools/patch_litex_server_for_openocd.sh`,
`tools/validate_vexriscv_debug_workflow.sh`, and the previous version of this document) were
shelved into a named `git stash` before deletion.

To recover them:

```bash
git stash list                                   # find the entry labelled:
                                                 #   "legacy-jtagbone-firmware-debug backup"
git stash apply 'stash@{N}'                      # restore the files (use the matching index N)
```

> [!NOTE]
> A `git stash` is convenient but easy to lose (it is local-only and not shared). If you expect to
> need the legacy stack again, promote the backup to a durable branch or annotated tag, e.g.
> `git stash branch legacy/jtagbone-firmware-debug 'stash@{N}'`.
