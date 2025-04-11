# Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

import sys
import os 
from datetime import datetime

output_path = ""
start_year = 2024

scheme_default = [
    ("boot",         "mcuboot",        64),
    ("sys",          "sys",            64),
    ("slot0",        "image-0",        -1),
    ("slot1",        "image-1",        -1),
    ("slot0-lpcore", "image-0-lpcore", 32),
    ("slot1-lpcore", "image-1-lpcore", 32),
    ("storage",      "storage",        192),
    ("scratch",      "image-scratch",  124),
    ("coredump",     "coredump",       4)
]

scheme_amp = [
    ("boot",         "mcuboot",        64),
    ("sys",          "sys",            64),
    ("slot0",        "image-0",        -3),
    ("slot1",        "image-1",        -3),
    ("slot0-appcpu", "image-0-appcpu", -1),
    ("slot1-appcpu", "image-1-appcpu", -1),
    ("slot0-lpcore", "image-0-lpcore", 32),
    ("slot1-lpcore", "image-1-lpcore", 32),
    ("storage",      "storage",        192),
    ("scratch",      "image-scratch",  124),
    ("coredump",     "coredump",       4)
]

flash_sizes_mb = [2, 4, 8, 16, 32]
flash_offsets_kb = [0, 4]
default_flash_mb = 4

def make_layout(flash_size_mb, flash_offset_kb, scheme):
    """
    Create a valid partitions layout based on provided scheme, flash size and flash offset.

    :param flash_size_mb: Total size of flash in MBytes
    :param flash_offset_kb: Flash start offset in kBytes
    :param scheme: Partition scheme used.
    """

    fixed_size = 0
    sum_weights = 0
    flash_size = flash_size_mb * 1024 * 1024
    offset = flash_offset_kb * 1024

    for i, (_, _, size) in enumerate(scheme):
        if size < 0:
            sum_weights += abs(size)
        else:
            fixed_size += size * 1024

    variable_size = flash_size - fixed_size
    if (variable_size % 0x10000) != 0:
        print("Variable size area must be in orders of 64kB due to the cache mappings. Correct the size of fixed allocations and try again.")
        sys.exit(1)

    variable_blocks = variable_size / 0x10000

    variable_alloc = []
    allocated_blocks = 0
    for i, (n, l, w) in enumerate(scheme):
        if w < 0:
            alloc = abs(w) * variable_blocks // sum_weights
            allocated_blocks += alloc
            variable_alloc.append([alloc, i])

    leftover_blocks = variable_blocks - allocated_blocks

    if leftover_blocks % 2 != 0:
        print("Odd number of leftower blocks cannot be distributed. Increase or decrease fixed partitions size by 64kB and try again.")
        sys.exit(1)
        
    if leftover_blocks:
        variable_alloc[0][0] += 1
        variable_alloc[1][0] += 1

    sum_kb = 0
    partition_layout = []

    for i, (n, l, s) in enumerate(scheme):
        size_kb = 0
        if (s < 0):
            for alloc, index in variable_alloc:
                if i == index:
                    size_kb = alloc * 64
        else:
            size_kb = s
        if i == 0:
            size_kb -= flash_offset_kb
        sum_kb += size_kb
        partition_layout.append((n, l, int(size_kb)))
    if sum_kb + flash_offset_kb != flash_size_mb * 1024:
        print("Something went wrong. Check the scheme and try again!")
        sys.exit(1)

    return partition_layout

def generate_partition_table_dtsi(flash_size_mb, flash_offset_kb, scheme, scheme_name):
    """
    Generate a DTSI file with a static flash partition layout.

    :param flash_size_mb: Total size of flash in MBytes
    :param flash_offset_kb: Flash start offset in kBytes
    :param scheme_name: Scheme name used in file name
    :param scheme: Partition scheme used.
    """

    print(f"Generate partition for {flash_size_mb}MB flash with offset 0x{flash_offset_kb*1024:x}kB")

    # Generate layout
    partitions = make_layout(flash_size_mb, flash_offset_kb, scheme)

    flash_size = flash_size_mb * 1024 * 1024
    current_offset = flash_offset_kb * 1024
    output = []

    output.append("/*")
    if start_year < datetime.now().year:
        output.append(f" * Copyright (c) {start_year}-{datetime.now().year} Espressif Systems (Shanghai) Co., Ltd.")
    else:
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

    for i, (name, label, size_kb) in enumerate(partitions):
        size = size_kb * 1024
        if current_offset + size > flash_size:
            print(f"Error: Partition {name} exceeds flash size. Aborting.")
            sys.exit(1)

        output.append("")
        output.append(f"\t\t{name.replace('-', '_')}_partition: partition@{current_offset:x} {{")
        output.append(f"\t\t\tlabel = \"{label}\";")
        output.append(f"\t\t\treg = <0x{current_offset:X} DT_SIZE_K({int(size_kb)})>;")
        output.append("\t\t};")
        print(f"{i}, {name:>16},{label:>16}, {current_offset:>8X}, DT_SIZE_K({size_kb})")
        current_offset += size

    output.append("\t};")
    output.append("};")
    output.append("")
    output.append(f"/* Remaining flash size is {int((flash_size - current_offset) / 1024)}kB")
    output.append(f" * Last used address is 0x{current_offset-1:X}")
    output.append(f" */")
    output.append("")

    output_file = f"{output_path}/partitions_{hex(flash_offset_kb * 1024)}_{scheme_name}_{int(flash_size_mb)}M.dtsi"

    with open(output_file, "w") as f:
        f.write("\n".join(output))

    print(f"Output file: {output_file}")

def generate_default_partition_table_dtsi(flash_size_mb, flash_offset_kb, scheme_name):

    output = []

    output.append("/*")
    if start_year < datetime.now().year:
        output.append(f" * Copyright (c) {start_year}-{datetime.now().year} Espressif Systems (Shanghai) Co., Ltd.")
    else:
        output.append(f" * Copyright (c) {datetime.now().year} Espressif Systems (Shanghai) Co., Ltd.")
    output.append(" *")
    output.append(" * SPDX-License-Identifier: Apache-2.0")
    output.append(" */")
    output.append("")
    output.append(f"#include <espressif/partitions_0x{(flash_offset_kb*1024):X}_{str(scheme_name)}_{int(flash_size_mb)}M.dtsi>")
    output.append("")

    output_file = f"{output_path}/partitions_{hex(offset_kb * 1024)}_{scheme_name}.dtsi"

    with open(output_file, "w") as f:
        f.write("\n".join(output))

    print(f"Output file: {output_file}")

if __name__ == "__main__":

    output_path = os.getcwd()

    if len(sys.argv) > 1:
        output_path = sys.argv[1]

    output_path = os.path.realpath(output_path)

    for offset_kb in flash_offsets_kb:
        for flash_mb in flash_sizes_mb:
            generate_partition_table_dtsi(flash_mb, offset_kb, scheme_default, "default")
        generate_default_partition_table_dtsi(default_flash_mb, offset_kb, "default")

    for offset_kb in flash_offsets_kb:
        for flash_mb in flash_sizes_mb:
            generate_partition_table_dtsi(flash_mb, offset_kb, scheme_amp, "amp")
        generate_default_partition_table_dtsi(default_flash_mb, offset_kb, "amp")

