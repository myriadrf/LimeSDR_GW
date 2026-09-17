#!/usr/bin/env python3
"""
program_board.py

Cross-platform board programming helper for Ubuntu and Windows.

Current features:
  - Program FPGA SPI flash using openFPGALoader
  - Program FPGA SRAM directly using openFPGALoader
  - Board-specific config for:
      * FPGA part
      * default JTAG frequency
      * supported cables
      * default cable
      * flash offsets
      * per-image openFPGALoader options

Examples:

  Flash user image using board default cable, ft2232:
    python3 program_board.py bitstream/hipersdr_44xx/hipersdr_44xx_user.bin \
        --board hipersdr_44xx \
        --target flash \
        --image user

  Flash user image using Digilent HS2:
    python3 program_board.py bitstream/hipersdr_44xx/hipersdr_44xx_user.bin \
        --board hipersdr_44xx \
        --target flash \
        --image user \
        --cable digilent_hs2

  Flash golden image:
    python3 program_board.py bitstream/hipersdr_44xx/hipersdr_44xx_golden.bin \
        --board hipersdr_44xx \
        --target flash \
        --image gold \
        --cable digilent_hs2

  SRAM programming:
    python3 program_board.py bitstream/hipersdr_44xx/hipersdr_44xx.bit \
        --board hipersdr_44xx \
        --target sram \
        --cable digilent_hs2

  Dry run:
    python3 program_board.py bitstream/hipersdr_44xx/hipersdr_44xx_user.bin \
        --board hipersdr_44xx \
        --target flash \
        --image user \
        --dry-run
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


SUPPORTED_TARGETS = ("flash", "sram")


BOARD_CONFIG = {
    "hipersdr_44xx": {
        "fpga_part": "xcau15p-ffvb676",
        "default_freq": "20000000",

        "cables": {
            "default": "ft2232",
            "supported": ("ft2232", "digilent_hs2"),
        },

        "targets": {
            "flash": {
                "allowed_extensions": (".bin",),

                "images": {
                    "gold": {
                        # Your gold command has no explicit offset.
                        "offset": None,
                        "verbose_level": None,
                    },

                    "user": {
                        # Your user command:
                        #   --offset 0x00500000 --verbose-level 2
                        "offset": "0x00500000",
                        "verbose_level": "2",
                    },
                },
            },

            "sram": {
                # For Xilinx SRAM programming, .bit is normally preferred.
                "allowed_extensions": (".bit", ".bin"),
                "verbose_level": None,
            },
        },
    },

    "limesdr_xtrx": {
        # Fill this with the exact openFPGALoader FPGA part string if needed.
        #
        # Example format:
        #   "xc7a50t-csg325"
        #
        # If None, --fpga-part is not passed.
        "fpga_part": None,
        "default_freq": None,

        "cables": {
            "default": "ft2232",
            "supported": ("ft2232", "digilent_hs2"),
        },

        "targets": {
            "flash": {
                "allowed_extensions": (".bin",),

                "images": {
                    "gold": {
                        "offset": None,
                        "verbose_level": None,
                    },

                    "user": {
                        "offset": None,
                        "verbose_level": None,
                    },
                },
            },

            "sram": {
                "allowed_extensions": (".bit", ".bin"),
                "verbose_level": None,
            },
        },
    },
}


SUPPORTED_BOARDS = tuple(BOARD_CONFIG.keys())
SUPPORTED_CABLES = tuple(
    sorted(
        {
            cable
            for board_cfg in BOARD_CONFIG.values()
            for cable in board_cfg["cables"]["supported"]
        }
    )
)


def format_command(cmd: list[str]) -> str:
    """Format command for readable terminal output."""
    if os.name == "nt":
        return subprocess.list2cmdline(cmd)

    return " ".join(shlex.quote(arg) for arg in cmd)


def find_openfpgaloader(explicit_path: str | None) -> str:
    """Find openFPGALoader executable."""
    if explicit_path:
        path = Path(explicit_path)

        if not path.exists():
            raise FileNotFoundError(f"openFPGALoader not found: {path}")

        if path.is_dir():
            raise FileNotFoundError(
                f"openFPGALoader path points to a directory, not executable: {path}"
            )

        return str(path)

    exe = shutil.which("openFPGALoader")
    if exe:
        return exe

    exe = shutil.which("openFPGALoader.exe")
    if exe:
        return exe

    raise FileNotFoundError(
        "openFPGALoader was not found in PATH. "
        "Install it or pass --openfpgaloader <path-to-openFPGALoader>."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Program FPGA boards using openFPGALoader."
    )

    parser.add_argument(
        "bitstream",
        type=Path,
        help="Input bitstream/image file. Usually .bin for flash, .bit for SRAM.",
    )

    parser.add_argument(
        "--board",
        choices=SUPPORTED_BOARDS,
        required=True,
        help="Target board.",
    )

    parser.add_argument(
        "--target",
        choices=SUPPORTED_TARGETS,
        default="flash",
        help="Programming target. Default: flash.",
    )

    image_group = parser.add_mutually_exclusive_group()

    image_group.add_argument(
        "--image",
        choices=("user", "gold"),
        default=None,
        help="Flash image slot. Valid only with --target flash. Default: user.",
    )

    # Backward-compatible shortcuts.
    image_group.add_argument(
        "--user",
        action="store_const",
        const="user",
        dest="image_shortcut",
        help="Shortcut for --image user. Valid only with --target flash.",
    )

    image_group.add_argument(
        "--gold",
        action="store_const",
        const="gold",
        dest="image_shortcut",
        help="Shortcut for --image gold. Valid only with --target flash.",
    )

    parser.add_argument(
        "--cable",
        choices=SUPPORTED_CABLES,
        default=None,
        help=(
            "JTAG cable to use. "
            "If omitted, board default cable from BOARD_CONFIG is used."
        ),
    )

    parser.add_argument(
        "--freq",
        type=str,
        default=None,
        help=(
            "Override JTAG frequency in Hz, for example 20000000. "
            "If omitted, board default is used."
        ),
    )

    parser.add_argument(
        "--verbose-level",
        type=str,
        default=None,
        help=(
            "Override openFPGALoader verbose level. "
            "If omitted, board/image default is used."
        ),
    )

    parser.add_argument(
        "--verify",
        action="store_true",
        help="Verify SPI flash after programming. Valid only with --target flash.",
    )

    parser.add_argument(
        "--reset",
        action="store_true",
        help="Add --reset to openFPGALoader command.",
    )

    parser.add_argument(
        "--openfpgaloader",
        default=None,
        help="Optional explicit path to openFPGALoader executable.",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print command but do not execute it.",
    )

    return parser.parse_args()


def resolve_image_arg(args: argparse.Namespace) -> str | None:
    """Resolve --image, --user, --gold into one image name."""
    image = args.image

    if getattr(args, "image_shortcut", None):
        image = args.image_shortcut

    if args.target == "flash":
        return image or "user"

    if image is not None:
        raise ValueError("--image, --user and --gold are valid only with --target flash")

    return None


def resolve_cable_arg(args: argparse.Namespace) -> str:
    """Resolve cable from CLI argument or board default."""
    board_cfg = BOARD_CONFIG[args.board]
    cable_cfg = board_cfg["cables"]

    cable = args.cable or cable_cfg["default"]
    supported = cable_cfg["supported"]

    if cable not in supported:
        supported_str = ", ".join(supported)
        raise ValueError(
            f"cable '{cable}' is not supported by board '{args.board}'. "
            f"Supported cables: {supported_str}"
        )

    return cable


def validate_board_config() -> None:
    """Catch config mistakes early."""
    for board, board_cfg in BOARD_CONFIG.items():
        cable_cfg = board_cfg["cables"]
        default_cable = cable_cfg["default"]
        supported_cables = cable_cfg["supported"]

        if default_cable not in supported_cables:
            raise ValueError(
                f"BOARD_CONFIG error for {board}: default cable '{default_cable}' "
                f"is not listed in supported cables {supported_cables}"
            )


def validate_bitstream_file(
    bitstream: Path,
    board: str,
    target: str,
    image: str | None,
) -> None:
    """Validate input file path and extension."""
    if not bitstream.exists():
        raise FileNotFoundError(f"file does not exist: {bitstream}")

    if not bitstream.is_file():
        raise ValueError(f"not a regular file: {bitstream}")

    board_cfg = BOARD_CONFIG[board]
    target_cfg = board_cfg["targets"][target]

    allowed_extensions = target_cfg["allowed_extensions"]
    suffix = bitstream.suffix.lower()

    if suffix not in allowed_extensions:
        allowed = ", ".join(allowed_extensions)

        if target == "flash":
            raise ValueError(
                f"invalid file extension for flash programming: {bitstream.name}. "
                f"Allowed extensions for {board}/{target}/{image}: {allowed}"
            )

        raise ValueError(
            f"invalid file extension for SRAM programming: {bitstream.name}. "
            f"Allowed extensions for {board}/{target}: {allowed}"
        )


def build_openfpgaloader_command(
    loader: str,
    bitstream: Path,
    board: str,
    target: str,
    image: str | None,
    cable: str,
    freq_override: str | None,
    verbose_override: str | None,
    verify: bool,
    reset: bool,
) -> list[str]:
    board_cfg = BOARD_CONFIG[board]
    target_cfg = board_cfg["targets"][target]

    fpga_part = board_cfg["fpga_part"]
    freq = freq_override if freq_override is not None else board_cfg["default_freq"]

    cmd = [loader]

    if fpga_part:
        cmd += ["--fpga-part", fpga_part]

    cmd += ["--cable", cable]

    if freq:
        cmd += ["--freq", freq]

    if target == "flash":
        if image is None:
            raise ValueError("internal error: flash target requires image")

        image_cfg = target_cfg["images"][image]

        offset = image_cfg["offset"]

        verbose_level = (
            verbose_override
            if verbose_override is not None
            else image_cfg["verbose_level"]
        )

        cmd += [
            "--write-flash",
            "--bitstream",
            str(bitstream),
        ]

        if offset:
            cmd += ["--offset", offset]

        if verbose_level:
            cmd += ["--verbose-level", verbose_level]

        if verify:
            cmd.append("--verify")

    elif target == "sram":
        if verify:
            raise ValueError("--verify is currently supported only with --target flash")

        verbose_level = (
            verbose_override
            if verbose_override is not None
            else target_cfg["verbose_level"]
        )

        cmd += [
            "--bitstream",
            str(bitstream),
        ]

        if verbose_level:
            cmd += ["--verbose-level", verbose_level]

    else:
        raise ValueError(f"unsupported target: {target}")

    if reset:
        cmd.append("--reset")

    return cmd


def print_summary(
    args: argparse.Namespace,
    image: str | None,
    cable: str,
    cmd: list[str],
) -> None:
    board_cfg = BOARD_CONFIG[args.board]
    target_cfg = board_cfg["targets"][args.target]

    print(f"Board        : {args.board}")
    print(f"Target       : {args.target}")
    print(f"FPGA part    : {board_cfg['fpga_part']}")
    print(f"Frequency    : {args.freq or board_cfg['default_freq']}")
    print(f"Cable        : {cable}")
    print(f"File         : {args.bitstream}")

    if args.target == "flash":
        image_cfg = target_cfg["images"][image]
        print(f"Image        : {image}")
        print(f"Offset       : {image_cfg['offset']}")
        print(f"Verify       : {args.verify}")

    print(f"Reset        : {args.reset}")
    print(f"Command      : {format_command(cmd)}")


def main() -> int:
    try:
        validate_board_config()

        args = parse_args()

        image = resolve_image_arg(args)
        cable = resolve_cable_arg(args)

        validate_bitstream_file(
            bitstream=args.bitstream,
            board=args.board,
            target=args.target,
            image=image,
        )

        loader = find_openfpgaloader(args.openfpgaloader)

        cmd = build_openfpgaloader_command(
            loader=loader,
            bitstream=args.bitstream,
            board=args.board,
            target=args.target,
            image=image,
            cable=cable,
            freq_override=args.freq,
            verbose_override=args.verbose_level,
            verify=args.verify,
            reset=args.reset,
        )

        print_summary(args, image, cable, cmd)

        if args.dry_run:
            return 0

        result = subprocess.run(cmd)

        if result.returncode != 0:
            print(
                f"ERROR: openFPGALoader failed with exit code {result.returncode}",
                file=sys.stderr,
            )
            return result.returncode

        print("Programming completed successfully.")
        return 0

    except KeyboardInterrupt:
        print("\nERROR: interrupted by user", file=sys.stderr)
        return 130

    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
