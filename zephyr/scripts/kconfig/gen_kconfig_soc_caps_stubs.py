#!/usr/bin/env python3
# Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

"""
Generate zephyr/Kconfig.soc_caps_stubs from all Kconfig.soc_caps.in files.

Collects every SOC_* symbol across all chips and emits a bare stub
(type only, no default) so kconfiglib always sees the symbol defined.

Usage:
    python3 gen_kconfig_soc_caps_stubs.py [--hal-root <path>]
"""

import argparse
import re
import sys
from datetime import datetime
from pathlib import Path


SOC_CAPS_GLOB = "components/soc/*/include/soc/Kconfig.soc_caps.in"
OUTPUT_FILE = "zephyr/scripts/kconfig/Kconfig.soc_caps_stubs"

HEADER = """\
# Copyright (c) {year} Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# AUTO-GENERATED — do not edit manually.
# Regenerate with: zephyr/scripts/kconfig/gen_kconfig_soc_caps_stubs.py

"""


def parse_args():
    script_dir = Path(__file__).resolve().parent
    default_hal_root = script_dir.parent.parent.parent

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hal-root", type=Path, default=default_hal_root,
                        help="Path to hal/espressif root (default: %(default)s)")
    return parser.parse_args()


def collect_symbols(hal_root):
    """
    Parse all Kconfig.soc_caps.in files and return an ordered dict of
    {symbol_name: type_string} sorted by symbol name.
    """
    symbols = {}

    for caps_file in sorted(hal_root.glob(SOC_CAPS_GLOB)):
        sym = None
        for line in caps_file.read_text(encoding="utf-8").splitlines():
            m = re.match(r'^config\s+(\w+)', line)
            if m:
                sym = m.group(1)
                continue
            if sym is not None:
                m = re.match(r'^\s+(bool|int|hex|string)\b', line)
                if m:
                    if sym not in symbols:
                        symbols[sym] = m.group(1)
                    sym = None
                elif line.strip() and not line.startswith('#'):
                    sym = None

    return dict(sorted(symbols.items()))


def write_stubs(output_path, symbols):
    lines = [HEADER.format(year=datetime.now().year)]

    for name, ktype in symbols.items():
        lines.append(f"config {name}\n\t{ktype}\n")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    hal_root = args.hal_root.resolve()

    if not hal_root.is_dir():
        print(f"error: hal-root not found: {hal_root}", file=sys.stderr)
        sys.exit(1)

    symbols = collect_symbols(hal_root)
    if not symbols:
        print("error: no symbols found — check hal-root path", file=sys.stderr)
        sys.exit(1)

    output_path = hal_root / OUTPUT_FILE
    write_stubs(output_path, symbols)

    print(f"wrote {len(symbols)} stub definitions → {output_path}")


if __name__ == "__main__":
    main()
