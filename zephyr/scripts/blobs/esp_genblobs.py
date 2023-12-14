#!/usr/bin/python3
# Copyright (c) 2022 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
import shutil
import ntpath
import hashlib
from pathlib import Path

# This relies on this file being in hal_espressif/zephyr/scripts/esp_genblobs.py
# If you move this file, you'll break it, so be careful.
MODULE_PATH = Path(Path(__file__).resolve().parents[2], "module.yml")
SUBMODULES = Path(Path(__file__).resolve().parents[1], "submodules.txt")

socs = ["esp32", "esp32s2", "esp32c3", "esp32s3", "esp32c6"]

module_yaml = """\
name: hal_espressif
build:
  cmake: zephyr
  kconfig: zephyr/Kconfig
  settings:
    dts_root: .
blobs:"""

blob_item = '''
  - path: lib/{SOC}/{FILENAME}
    sha256: {SHA256}
    type: lib
    version: '1.0'
    license-path: zephyr/blobs/license.txt
    url: {URL}/raw/{REV}/{SOC}/{FILENAME}
    description: "Binary libraries supporting the ESP32 series RF subsystems"
    doc-url: {URL_BASE}'''


def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


def download_repositories():
    path = os.path.dirname(os.path.abspath(__file__))
    with open(SUBMODULES) as f:
        for submodule in f:
            git_rev, git_dir, git_url = submodule.split()
            folder = Path(path, "temp", git_dir)
            if not folder.exists():
                print("Cloning into {}".format(folder))
                cmd_exec(("git", "clone", git_url, folder, "--quiet"), cwd=path)
            print("Checking out revision {} at {}".format(git_rev, folder))
            cmd_exec(("git", "-C", folder, "fetch"), cwd=path)
            cmd_exec(("git", "-C", folder, "checkout", git_rev, "--quiet"), cwd=path)


def clean_up():
    print("deleted temporary files..")
    path = os.path.dirname(os.path.abspath(__file__))
    folder = Path(path, "temp")
    shutil.rmtree(folder)


def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)


def get_file_sha256(path):
    with open(path,"rb") as f:
        f_byte = f.read()
        result = hashlib.sha256(f_byte)
        return result.hexdigest()


def generate_blob_list():
    file_out = module_yaml

    path = os.path.dirname(os.path.abspath(__file__))
    with open(SUBMODULES) as f:
        for submodule in f:
            git_rev, git_dir, git_url = submodule.split()

            for s in socs:
                folder = Path(path, "temp", git_dir, s)
                pathlist = []
                pathlist.extend(Path(folder).glob('**/*.a'))
                pathlist.extend(Path(folder).glob('**/*.bin'))
                for item in pathlist:
                    path_in_str = str(item)
                    filename = path_leaf(path_in_str)
                    sha256 = get_file_sha256(path_in_str)
                    file_out += blob_item.format(SOC=s,
                                                 FILENAME=filename,
                                                 SHA256=sha256,
                                                 URL=git_url,
                                                 REV=git_rev,
                                                 URL_BASE=git_url)
    file_out += "\r\n"

    try:
        os.remove(filename)
    except OSError:
        pass

    with open(MODULE_PATH, "w+") as f:
        f.write(file_out)
        f.close()

    print("module.yml updated!")


if __name__ == '__main__':
    download_repositories()
    generate_blob_list()
    clean_up()
