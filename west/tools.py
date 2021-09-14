# Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

'''tools.py

Espressif west extension adds some functions and features
to enable proper submodules update, toolchain installation
and serial port monitor logging.'''

import os
import platform
import subprocess
import sys
import yaml
from pathlib import Path

from textwrap import dedent
from west.commands import WestCommand
from west.configuration import config
from west import log

# This relies on this file being in hal_espressif/west/tools.py
# If you move this file, you'll break it, so be careful.
THIS_ZEPHYR = Path(__file__).parents[4] / 'zephyr'
ZEPHYR_BASE = Path(os.environ.get('ZEPHYR_BASE', THIS_ZEPHYR))

sys.path.insert(0, os.path.join(ZEPHYR_BASE, "scripts", "west_commands"))

from build_helpers import is_zephyr_build, find_build_dir  # noqa: E402

ESP_IDF_REMOTE = "https://github.com/zephyrproject-rtos/hal_espressif"


def cmd_check(cmd, cwd=None, stderr=subprocess.STDOUT):
    return subprocess.check_output(cmd, cwd=cwd, stderr=stderr)


def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


def get_esp_serial_port(module_path):
    try:
        import serial.tools.list_ports
        esptool_path = os.path.join(module_path, 'components', 'esptool_py', 'esptool')
        sys.path.insert(0, esptool_path)
        import esptool
        ports = list(sorted(p.device for p in serial.tools.list_ports.comports()))
        # high baud rate could cause the failure of creation of the connection
        esp = esptool.get_default_connected_device(serial_list=ports, port=None, connect_attempts=4,
                                                   initial_baud=115200)
        if esp is None:
            log.die("No serial ports found. Connect a device, or use '-p PORT' "
                    "option to set a specific port.")

        serial_port = esp.serial_port
        esp._port.close()

        return serial_port
    except Exception as e:
        log.die("Error detecting ESP serial port: {}".format(e))
        return None


def get_build_dir(args, die_if_none=True):
    # Get the build directory for the given argument list and environment.

    guess = config.get('build', 'guess-dir', fallback='never')
    guess = guess == 'runners'
    dir = find_build_dir(None, guess)

    if dir and is_zephyr_build(dir):
        return dir
    elif die_if_none:
        msg = 'could not find build directory and '
        if dir:
            msg = msg + 'neither {} nor {} are zephyr build directories.'
        else:
            msg = msg + ('{} is not a build directory and the default build '
                         'directory cannot be determined. ')
        log.die(msg.format(os.getcwd(), dir))
    else:
        return None


def get_build_elf_path():
    def runners_yaml_path(build_dir):
        ret = Path(build_dir) / 'zephyr' / 'runners.yaml'
        if not ret.is_file():
            log.die("could not find build configuration")
        return ret

    def load_runners_yaml(path):
        # Load runners.yaml and convert to Python object.

        try:
            with open(path, 'r') as f:
                content = yaml.safe_load(f.read())
        except Exception as e:
            log.die("runners.yaml file open error: {}".format(e))

        if not content.get('runners'):
            log.wrn("no pre-configured runners in {}; "
                    "this probably won't work".format(path))

        return content

    build_dir = get_build_dir(None)
    yaml_path = runners_yaml_path(build_dir)
    runners_yaml = load_runners_yaml(yaml_path)

    return Path(build_dir) / 'zephyr' / runners_yaml['config']['elf_file']


class Tools(WestCommand):

    def __init__(self):
        super().__init__(
            'espressif',
            # Keep this in sync with the string in west-commands.yml.
            'Espressif tools for west framework.',
            dedent('''
            This interface allows updating hal_espressif submodules,
            installing Espressif toolchain and open serial monitor for
            Espressif SoC devices.'''),
            accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('command', choices=['install', 'update', 'monitor'],
                            help='install espressif toolchain or fetch submodules')

        # monitor arguments
        group = parser.add_argument_group('monitor optional arguments')
        group.add_argument('-b', '--baud', default="115200", help='Serial port baud rate')
        group.add_argument('-p', '--port', help='Serial port address')
        group.add_argument('-e', '--elf', help='ELF file')

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
        elif args.command == "monitor":
            self.monitor(module_path, args)

    def update(self, module_path):
        log.banner('updating ESP-IDF submodules..')

        # look for origin remote
        remote_name = cmd_check(("git", "remote"), cwd=module_path).decode('utf-8')

        if "origin" not in remote_name:
            # add origin url
            cmd_exec(("git", "remote", "add", "origin", ESP_IDF_REMOTE), cwd=module_path)
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
            toolchain_path = os.path.join(os.environ['USERPROFILE'], '.espressif', 'tools', 'zephyr')
            cmd = "set"
        else:
            cmd_exec(("./tools/idf_tools.py", "--tools-json=tools/zephyr_tools.json", "install"),
                     cwd=module_path)
            toolchain_path = os.path.join(os.environ['HOME'], '.espressif', 'tools', 'zephyr')
            cmd = "export"

        log.banner('downloading ESP-IDF tools completed')

        log.inf("The toolchain has been downloaded to {}".format(toolchain_path))
        log.inf("Export or set the following variables into the environment:")
        log.inf("{} ESPRESSIF_TOOLCHAIN_PATH=\"{}\"".format(cmd, toolchain_path))
        log.inf("{} ZEPHYR_TOOLCHAIN_VARIANT=\"espressif\"".format(cmd))

    def monitor(self, module_path, args):

        elf_path = args.elf
        if elf_path:
            elf_path = os.path.abspath(elf_path)
        else:
            # get build elf file path
            elf_path = get_build_elf_path()

        if not os.path.exists(elf_path):
            log.die("Unable to determine ELF file path. Rebuild the project "
                    "or add '-e path/to/elf' argument to continue.")

        esp_port = args.port
        if not esp_port:
            # detect usb port using esptool
            esp_port = get_esp_serial_port(module_path)

        if platform.system() == 'Windows':
            cmd_exec(("python.exe", "tools/idf_monitor.py", "-p", esp_port,
                     "-b", args.baud, elf_path), cwd=module_path)
        else:
            cmd_exec(("./tools/idf_monitor.py", "-p", esp_port, "-b", args.baud, elf_path),
                     cwd=module_path)
