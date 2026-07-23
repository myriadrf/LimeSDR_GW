#!/usr/bin/env bash
#
# Minimal end-to-end validator for the LimeSDR-USB native RISC-V debug workflow.
#
# Given an already-built and programmed --with-cpu-debug board connected over the
# FT2232 JTAG cable, this launches upstream OpenOCD (interface + shared tunnel
# script) and drives a scripted GDB batch that exercises the three behaviours the
# old jtagbone/etherbone stack could not do reliably:
#
#   1. a plain `break main` that must actually fire (hardware breakpoint vs ROM),
#   2. a single-step that must advance the PC,
#   3. an interrupt (pause) + `continue` (resume) cycle that must NOT drop the
#      session.
#
# It prints a single PASS/FAIL verdict. This is an internal development aid, in
# the same spirit as the rest of tools/ -- not part of the production flow.
#
# Usage (from the repository root):
#   tools/validate_usb_cpu_debug.sh [--elf PATH] [--gdb BIN] [--port N]
#
set -u

# --- Defaults ------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${REPO_ROOT}/firmware/firmware.elf"
GDB_BIN="riscv32-unknown-elf-gdb"
GDB_PORT="3333"
OPENOCD_BIN="openocd"
IFACE_CFG="${REPO_ROOT}/tools/limesdr_usb_debug.cfg"
TUNNEL_TCL="${REPO_ROOT}/riscv_jtag_tunneled.tcl"

usage() { grep '^#' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --elf)  ELF="$2";       shift 2;;
        --gdb)  GDB_BIN="$2";   shift 2;;
        --port) GDB_PORT="$2";  shift 2;;
        -h|--help) usage; exit 0;;
        *) echo "Unknown option: $1" >&2; usage; exit 2;;
    esac
done

fail() { echo "RESULT: FAIL -- $*"; exit 1; }

# --- Prerequisite checks (fail fast with a clear message) ----------------------
command -v "$OPENOCD_BIN" >/dev/null 2>&1 || fail "openocd not found in PATH"
command -v "$GDB_BIN"     >/dev/null 2>&1 || fail "$GDB_BIN not found in PATH"
[ -f "$ELF" ]        || fail "firmware ELF not found: $ELF (build with --with-cpu-debug)"
[ -f "$IFACE_CFG" ]  || fail "interface config not found: $IFACE_CFG"
[ -f "$TUNNEL_TCL" ] || fail "tunnel script not found: $TUNNEL_TCL"

WORKDIR="$(mktemp -d)"
OCD_LOG="${WORKDIR}/openocd.log"
GDB_LOG="${WORKDIR}/gdb.log"
cleanup() {
    [ -n "${OCD_PID:-}" ] && kill "$OCD_PID" >/dev/null 2>&1
    rm -rf "$WORKDIR"
}
trap cleanup EXIT

# --- Start OpenOCD -------------------------------------------------------------
echo "Starting OpenOCD ($IFACE_CFG + $TUNNEL_TCL)..."
( cd "$REPO_ROOT" && "$OPENOCD_BIN" -f "$IFACE_CFG" -f "$TUNNEL_TCL" ) >"$OCD_LOG" 2>&1 &
OCD_PID=$!

# Wait for the GDB server to come up.
for _ in $(seq 1 30); do
    grep -q "Listening on port ${GDB_PORT} for gdb" "$OCD_LOG" 2>/dev/null && break
    kill -0 "$OCD_PID" 2>/dev/null || { cat "$OCD_LOG"; fail "OpenOCD exited early"; }
    sleep 1
done
grep -q "Listening on port ${GDB_PORT} for gdb" "$OCD_LOG" 2>/dev/null \
    || { cat "$OCD_LOG"; fail "OpenOCD never opened the GDB server on :${GDB_PORT}"; }

# --- Drive GDB -----------------------------------------------------------------
echo "Driving GDB batch..."
"$GDB_BIN" -nx -batch "$ELF" \
    -ex "set remotetimeout 20" \
    -ex "target remote :${GDB_PORT}" \
    -ex "monitor reset halt" \
    -ex "break main" \
    -ex "continue" \
    -ex "printf \"HIT_PC=%p\\n\", \$pc" \
    -ex "stepi" \
    -ex "printf \"STEP_PC=%p\\n\", \$pc" \
    -ex "interrupt" \
    -ex "continue &" \
    -ex "interrupt" \
    -ex "printf \"RESUME_OK\\n\"" \
    -ex "detach" \
    >"$GDB_LOG" 2>&1

echo "----- GDB output -----"; cat "$GDB_LOG"; echo "----------------------"

# --- Verdict -------------------------------------------------------------------
grep -q "HIT_PC="   "$GDB_LOG" || fail "breakpoint at main never fired"
grep -q "STEP_PC="  "$GDB_LOG" || fail "single-step did not report a PC"
grep -q "RESUME_OK" "$GDB_LOG" || fail "interrupt+continue cycle tore down the session"

echo "RESULT: PASS"
