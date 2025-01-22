# Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

import sys
from datetime import datetime
from collections import namedtuple

# where the files ought to be copied
output_path_prefix = "dts/common/espressif/"

# Define a namedtuple to represent the structure
partition_scheme = namedtuple("partition_scheme", ["size_mb", "partitions"])

partitions_default = [
    partition_scheme(
        size_mb = 2,
        partitions = [
            ("boot",         128),
            ("slot0",        832),
            ("slot1",        832),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 4,
        partitions = [
            ("boot",         128),
            ("slot0",        1856),
            ("slot1",        1856),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 8,
        partitions = [
            ("boot",         128),
            ("slot0",        3904),
            ("slot1",        3904),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 16,
        partitions = [
            ("boot",         128),
            ("slot0",        8000),
            ("slot1",        8000),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 32,
        partitions = [
            ("boot",         128),
            ("slot0",        16192),
            ("slot1",        16192),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
]

partitions_amp = [
    partition_scheme(
        size_mb = 2,
        partitions = [
            ("boot",         128),
            ("slot0",        576),
            ("slot1",        576),
            ("slot0-appcpu", 256),
            ("slot1-appcpu", 256),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 4,
        partitions = [
            ("boot",         128),
            ("slot0",        1344),
            ("slot1",        1344),
            ("slot0-appcpu", 512),
            ("slot1-appcpu", 512),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 8,
        partitions = [
            ("boot",         128),
            ("slot0",        3136),
            ("slot1",        3136),
            ("slot0-appcpu", 768),
            ("slot1-appcpu", 768),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 16,
        partitions = [
            ("boot",         128),
            ("slot0",        6080),
            ("slot1",        6080),
            ("slot0-appcpu", 1920),
            ("slot1-appcpu", 1920),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
    partition_scheme(
        size_mb = 32,
        partitions = [
            ("boot",         128),
            ("slot0",        12352),
            ("slot1",        12352),
            ("slot0-appcpu", 3840),
            ("slot1-appcpu", 3840),
            ("slot0-lpcore", 32),
            ("slot1-lpcore", 32),
            ("storage",      128),
            ("coredump",     4)
        ]
    ),
]

def generate_partition_table_dtsi(scheme, flash_offset_kb, scheme_name):
    """
    Generate a DTSI file with a static flash partition layout.

    :param scheme: Partition scheme used.
    """
    flash_size = scheme.size_mb * 1024 * 1024
    current_offset = flash_offset_kb * 1024
    output = []

    output.append("/*")
    output.append(f" * Copyright (c) {datetime.now().year} Espressif Systems (Shanghai) Co., Ltd.")
    output.append(" *")
    output.append(" * SPDX-License-Identifier: Apache-2.0")
    output.append(" */")
    output.append("")
    output.append("&flash0 {")
    output.append("\tpartitions {")
    output.append("\t\tcompatible = \"fixed-partitions\";")
    output.append("\t\t#address-cells = <1>;")
    output.append("\t\t#size-cells = <1>;")
    output.append("")

    for name, size_kb in scheme.partitions:
        if name == "boot":
            size_kb -= flash_offset_kb
        size = size_kb * 1024
        if current_offset + size > flash_size:
            print(f"Error: Partition {name} exceeds flash size. Aborting.")
            sys.exit(1)

        output.append(f"\t\t{name.replace('-', '_')}_partition: partition@{current_offset:X} {{")
        output.append(f"\t\t\tlabel = \"{name}\";")
        output.append(f"\t\t\treg = <0x{current_offset:X} DT_SIZE_K({int(size_kb)})>;")
        output.append("\t\t};")
        output.append("")
        current_offset += size

    output.append("\t};")
    output.append("};")
    output.append("")
    output.append(f"/* Remaining flash size is {int((flash_size - current_offset) / 1024)}kB */")

    filename = f"partitions_{hex(flash_offset_kb * 1024)}_{scheme_name}_{int(scheme.size_mb)}M.dtsi"

    # Write the generated content to the output file
    with open(filename, "w") as f:
        f.write("\n".join(output))
    
    print(f"DTSI file generated successfully: {filename}")

def generate_default_partition_table_dtsi(offset_kb, scheme_name, flash_size_mb):

    output = []

    output.append("/*")
    output.append(f" * Copyright (c) {datetime.now().year} Espressif Systems (Shanghai) Co., Ltd.")
    output.append(" *")
    output.append(" * SPDX-License-Identifier: Apache-2.0")
    output.append(" */")
    output.append("")
    output.append(f"#include <espressif/partitions_0x{(offset_kb*1024):X}_{str(scheme_name)}_{int(flash_size_mb)}M.dtsi>")

    filename = f"partitions_{hex(offset_kb * 1024)}_{scheme_name}.dtsi"

    # Write the generated content to the output file
    with open(filename, "w") as f:
        f.write("\n".join(output))
    
    print(f"DTSI file generated successfully: {filename}")

# Example usage
if __name__ == "__main__":

    # default partitions with flash offset 0x0
    for p in partitions_default:
        generate_partition_table_dtsi(p, 0, "default")
    generate_default_partition_table_dtsi(0, "default", 4)

    # default partitions with flash offset 0x1000
    for p in partitions_default:
        generate_partition_table_dtsi(p, 4, "default")
    generate_default_partition_table_dtsi(4, "default", 4)

    # AMP partitions with flash offset 0x0
    for p in partitions_amp:
        generate_partition_table_dtsi(p, 0, "amp")
    generate_default_partition_table_dtsi(0, "amp", 4)

    # AMP partitions with flash offset 0x1000
    for p in partitions_amp:
        generate_partition_table_dtsi(p, 4, "amp")
    generate_default_partition_table_dtsi(4, "amp", 4)
