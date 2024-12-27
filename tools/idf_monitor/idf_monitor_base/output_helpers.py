# Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess
import sys
from typing import BinaryIO, Callable, Optional, Union  # noqa: F401

from west.commands import WestCommand
from west.configuration import config
from west import log

# This relies on this file being in hal_espressif/tools/idf_monitor_base/output_helpers.py
# If you move this file, you'll break it, so be careful.
from pathlib import Path
THIS_ZEPHYR = Path(__file__).parents[5] / 'zephyr'
ZEPHYR_BASE = Path(os.environ.get('ZEPHYR_BASE', THIS_ZEPHYR))

sys.path.insert(0, os.path.join(ZEPHYR_BASE, "scripts", "west_commands"))

from build_helpers import load_domains
from build_helpers import is_zephyr_build, find_build_dir  # noqa: E402
from runners.core import BuildConfiguration  # noqa: E402
from zcmake import CMakeCache


# ANSI terminal codes (if changed, regular expressions in LineMatcher need to be updated)
ANSI_RED = '\033[1;31m'
ANSI_YELLOW = '\033[0;33m'
ANSI_NORMAL = '\033[0m'


def color_print(message, color, newline='\n'):  # type: (str, str, Optional[str]) -> None
    """ Print a message to stderr with colored highlighting """
    sys.stderr.write('%s%s%s%s' % (color, message, ANSI_NORMAL, newline))


def normal_print(message):  # type: (str) -> None
    sys.stderr.write(ANSI_NORMAL + message)


def yellow_print(message, newline='\n'):  # type: (str, Optional[str]) -> None
    color_print(message, ANSI_YELLOW, newline)


def red_print(message, newline='\n'):  # type: (str, Optional[str]) -> None
    color_print(message, ANSI_RED, newline)


def lookup_pc_address(pc_addr, toolchain_prefix, elf_file):  # type: (str, str, str) -> Optional[str]
    # cmd = ['%saddr2line' % toolchain_prefix, '-pfiaC', '-e', elf_file, pc_addr]

    build_dir = find_build_dir(None, True)
    domain = load_domains(build_dir).get_default_domain()

    # build dir differs when sysbuild is used
    if domain.build_dir:
        build_dir = domain.build_dir

    cache = CMakeCache.from_build_dir(build_dir)

    # Zephyr: set toolchain from environment path
    toolchain_path = cache['CMAKE_ADDR2LINE']
    cmd = [toolchain_path, '-pfiaC', '-e', elf_file, pc_addr]

    try:
        translation = subprocess.check_output(cmd, cwd='.')
        if b'?? ??:0' not in translation:
            return translation.decode()
    except OSError as e:
        red_print('%s: %s' % (' '.join(cmd), e))
    return None
