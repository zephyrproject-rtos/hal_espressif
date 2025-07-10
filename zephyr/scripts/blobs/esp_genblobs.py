#!/usr/bin/python3
# Copyright (c) 2022-2025 Espressif Systems (Shanghai) Co., Ltd.
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

socs = ["esp32", "esp32s2", "esp32c2", "esp32c3", "esp32s3", "esp32c6"]

module_yaml = """\
name: hal_espressif
package-managers:
  pip:
    requirement-files:
      - zephyr/requirements.txt
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
    url: {URL}/raw/{REV}/{URL_PATH}
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


def add_blobs(file_out, soc, pathlist, git_url, git_rev, url_path_prefix):
    for item in pathlist:
        filename = path_leaf(str(item))
        sha256 = get_file_sha256(item)
        url_path = f"{url_path_prefix}/{filename}" if url_path_prefix else filename
        file_out += blob_item.format(
            SOC=soc,
            FILENAME=filename,
            SHA256=sha256,
            URL=git_url,
            REV=git_rev,
            URL_PATH=url_path,
            URL_BASE=git_url,
        )
    return file_out


def generate_blob_list():
    file_out = module_yaml

    path = os.path.dirname(os.path.abspath(__file__))
    with open(SUBMODULES) as f:
        for submodule in f:
            git_rev, git_dir, git_url = submodule.split()
            base_dir = Path(path, "temp", git_dir)
            found_soc = False

            for s in socs:
                soc_dir = base_dir / s
                if soc_dir.exists():
                    pathlist = list(soc_dir.glob('**/*.a')) + list(soc_dir.glob('**/*.bin'))
                    file_out = add_blobs(file_out, s, pathlist, git_url, git_rev, url_path_prefix=s)
                    found_soc = True

            if not found_soc:
                matched_soc = None
                for s in sorted(socs, key=len, reverse=True):
                    if f"/{s}" in f"/{git_dir.lower()}":
                        matched_soc = s
                        break

                if matched_soc:
                    print(f"[fallback] No subfolder found for {git_dir}, assuming SoC: {matched_soc}")
                    fallback_folder = base_dir
                    pathlist = list(fallback_folder.glob('**/*.a')) + list(fallback_folder.glob('**/*.bin'))
                    file_out = add_blobs(file_out, matched_soc, pathlist, git_url, git_rev, url_path_prefix=None)
                else:
                    print(f"[warning] Skipping {git_dir}, no SoC match found.")

    file_out += "\r\n"

    try:
        os.remove(MODULE_PATH)
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
