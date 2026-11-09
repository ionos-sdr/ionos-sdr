# SPDX-License-Identifier: MIT
# pio_fw_version.py - FW_VERSION from 'git describe' (INFO page, HOST-M1 VER?).
# Used as:  extra_scripts = pre:tools/pio_fw_version.py
import subprocess
Import("env")

def git_describe():
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=env.subst("$PROJECT_DIR"), stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return "unknown"

ver = git_describe()
env.Append(CPPDEFINES=[("FW_VERSION", env.StringifyMacro(ver))])
print("FW_VERSION =", ver)
