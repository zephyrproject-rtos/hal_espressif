# Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

'''update.py

Espressif west extension to retrieve esp-idf submodules.'''

import os
import stat
import shutil
import subprocess
import re
from pathlib import Path

from textwrap import dedent
from west.commands import WestCommand
from west import log

ESP_IDF_REMOTE = "https://github.com/zephyrproject-rtos/esp-idf"


def cmd_check(cmd, cwd=None, stderr=subprocess.STDOUT):
    return subprocess.check_output(cmd, cwd=cwd, stderr=stderr)

def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)

class GetSubModules(WestCommand):

    def __init__(self):
        super().__init__(
            'espressif',
            # Keep this in sync with the string in west-commands.yml.
            'download or update ESP-IDF submodules',
            dedent('''
            This command fetches all ESP-IDF submodules required
            for Espressif SoC devices framework.'''),
            accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('command', choices=['update'], 
                            help='retrieve esp-idf submodules')

        return parser

    def do_run(self, args, unknown_args):
        if args.command == "update":
            self.update()
        
    def update(self):
        log.banner('updating ESP-IDF submodules..')

        module_path = (
                Path(os.getenv("ZEPHYR_BASE")).absolute()
                / r".."
                / "modules"
                / "hal"
                / "esp-idf"
            )

        if not module_path.exists():
            log.die('cannot find esp-idf project in $ZEPHYR_BASE path')

        remote_output = cmd_check(("git", "remote", "-v"), 
                                    cwd=module_path).decode('utf-8')

        if not ESP_IDF_REMOTE in remote_output:
            cmd_exec(("git", "remote", "add", "origin", 
                        ESP_IDF_REMOTE), cwd=module_path)
        
        cmd_exec(("git", "submodule", "update", "--init", "--recursive"), 
                    cwd=module_path)

        log.banner('updating ESP-IDF submodules completed')