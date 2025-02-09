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

build_elf_path = None
baud_rate = None

def cmd_check(cmd, cwd=None, stderr=subprocess.STDOUT):
    return subprocess.check_output(cmd, cwd=cwd, stderr=stderr)


def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


def get_esp_serial_port(module_path):
    try:
        import serial.tools.list_ports
        esptool_path = os.path.join(module_path, 'tools', 'esptool_py')
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


def parse_runners_yaml():
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

    global build_elf_path
    global baud_rate

    build_dir = find_build_dir(None, True)
    domain = load_domains(build_dir).get_default_domain()

    # build dir differs when sysbuild is used
    if domain.build_dir:
        build_dir = domain.build_dir

    yaml_path = runners_yaml_path(build_dir)
    runners_yaml = load_runners_yaml(yaml_path)

    # get build elf file path
    build_elf_path = Path(build_dir) / 'zephyr' / runners_yaml['config']['elf_file']

    # parse specific arguments for the tool
    yaml_args = runners_yaml['args']['esp32']

    # Iterate through the list to find default baud rate
    for item in yaml_args:
        if item.startswith('--esp-monitor-baud='):
            baud_rate = item.split('=')[1]
            break

    # if specific default baud rate is not defined in runners.yaml, default to 115200
    if not baud_rate:
        baud_rate = "115200"

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

        parse_runners_yaml()

        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('command', choices=['monitor'],
                            help='open serial port based on esp-idf monitor')

        # monitor arguments
        group = parser.add_argument_group('monitor optional arguments')
        group.add_argument('-b', '--baud', default=baud_rate, help='Serial port baud rate')
        group.add_argument('-p', '--port', help='Serial port address')
        group.add_argument('-e', '--elf', help='ELF file')
        group.add_argument('-n', '--eol', default='CRLF', help='EOL to use')
        group.add_argument('-d', '--enable-address-decoding', action='store_true',
                           help='Enable address decoding in the monitor')

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
            elf_path = build_elf_path

        esp_port = args.port
        if not esp_port:
            # detect usb port using esptool
            esp_port = get_esp_serial_port(module_path)

        monitor_path = Path(module_path, "tools/idf_monitor/idf_monitor.py")
        cmd_path = Path(os.getcwd())

        cmd = [sys.executable, str(monitor_path), "-p", esp_port, "-b", args.baud, str(elf_path), "--eol", args.eol]

        # Adding "-d" argument for idf_monitor.py disables the address decoding
        if not args.enable_address_decoding:
            cmd.append("-d")

        cmd_exec(cmd, cwd=cmd_path)
