# Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

'''tools.py

Espressif west extension for serial port monitor logging.'''

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

from build_helpers import load_domains
from build_helpers import is_zephyr_build, find_build_dir  # noqa: E402
from runners.core import BuildConfiguration  # noqa: E402

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


def get_build_dir(path, die_if_none=True):
    # Get the build directory for the given argument list and environment.

    guess = config.get('build', 'guess-dir', fallback='never')
    guess = guess == 'runners'
    dir = find_build_dir(path, guess)

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
    domain = load_domains(build_dir).get_default_domain()

    # build dir differs when sysbuild is used
    if domain.name != 'app':
        build_dir = Path(build_dir) / domain.name
        build_dir = get_build_dir(build_dir)

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
            This interface allows having esp-idf monitor support.'''),
            accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('command', choices=['monitor'],
                            help='open serial port based on esp-idf monitor')

        # monitor arguments
        group = parser.add_argument_group('monitor optional arguments')
        group.add_argument('-b', '--baud', default="115200", help='Serial port baud rate')
        group.add_argument('-p', '--port', help='Serial port address')
        group.add_argument('-e', '--elf', help='ELF file')
        group.add_argument('-n', '--eol', default='CRLF', help='EOL to use')

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
            log.die('cannot find espressif hal in $ZEPHYR_BASE/../modules/hal/ path')

        if args.command == "monitor":
            self.monitor(module_path, args)

    def monitor(self, module_path, args):

        elf_path = args.elf
        if elf_path:
            elf_path = os.path.abspath(elf_path)
        else:
            # get build elf file path
            elf_path = get_build_elf_path()

        esp_port = args.port
        if not esp_port:
            # detect usb port using esptool
            esp_port = get_esp_serial_port(module_path)

        monitor_path = Path(module_path, "tools/idf_monitor.py")
        cmd_path = Path(os.getcwd())
        if platform.system() == 'Windows':
            cmd_exec(("python.exe", monitor_path, "-p", esp_port,
                     "-b", args.baud, elf_path, "--eol", args.eol), cwd=cmd_path)
        else:
            cmd_exec((sys.executable, monitor_path, "-p", esp_port, "-b", args.baud,
                      elf_path, "--eol", args.eol), cwd=cmd_path)
