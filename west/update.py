# Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

'''update.py

Espressif west extension to retrieve esp-idf submodules.'''

import os
import platform
import subprocess
from pathlib import Path

from textwrap import dedent
from west.commands import WestCommand
from west import log

ESP_IDF_REMOTE = "https://github.com/zephyrproject-rtos/hal_espressif"


def cmd_check(cmd, cwd=None, stderr=subprocess.STDOUT):
    return subprocess.check_output(cmd, cwd=cwd, stderr=stderr)


def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


class UpdateTool(WestCommand):

    def __init__(self):
        super().__init__(
            'espressif',
            # Keep this in sync with the string in west-commands.yml.
            'download toolchain or update ESP-IDF submodules',
            dedent('''
            This interface allows downloading Espressif toolchain
            or fetch ESP-IDF submodules required for
            Espressif SoC devices framework.'''),
            accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('command', choices=['install', 'update'],
                            help='install espressif toolchain or fetch submodules')

        return parser

    def do_run(self, args, unknown_args):

        module_path = (
            Path(os.getenv("ZEPHYR_BASE")).absolute()
            / r".."
            / "modules"
            / "hal"
            / "espressif"
        )

        if not module_path.exists():
            log.die('cannot find espressif project in $ZEPHYR_BASE path')

        if args.command == "update":
            self.update(module_path)
        elif args.command == "install":
            self.install(module_path)

    def update(self, module_path):
        log.banner('updating ESP-IDF submodules..')

        # look for origin remote
        remote_name = cmd_check(("git", "remote"),
                                cwd=module_path).decode('utf-8')

        if "origin" not in remote_name:
            # add origin url
            cmd_exec(("git", "remote", "add", "origin",
                     ESP_IDF_REMOTE), cwd=module_path)
        else:
            remote_url = cmd_check(("git", "remote", "get-url", "origin"),
                                   cwd=module_path).decode('utf-8')
            # update origin URL
            if ESP_IDF_REMOTE not in remote_url:
                cmd_exec(("git", "remote", "set-url", "origin",
                         ESP_IDF_REMOTE), cwd=module_path)

        cmd_exec(("git", "submodule", "update", "--init", "--recursive"),
                 cwd=module_path)

        log.banner('updating ESP-IDF submodules completed')

    def install(self, module_path):

        log.banner('downloading ESP-IDF tools..')

        if platform.system() == 'Windows':
            cmd_exec(("python.exe", "tools/idf_tools.py", "--tools-json=tools/zephyr_tools.json", "install"),
                     cwd=module_path)
        else:
            cmd_exec(("./tools/idf_tools.py", "--tools-json=tools/zephyr_tools.json", "install"),
                     cwd=module_path)

        log.banner('downloading ESP-IDF tools completed')
