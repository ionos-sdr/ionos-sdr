# SPDX-License-Identifier: MIT
#
# pio_sdkconfig_refresh.py — PlatformIO "pre" extra script for the hybrid
# (arduino, espidf) environments.
#
# THE PROBLEM IT SOLVES: ESP-IDF applies sdkconfig.*.defaults only when the
# generated sdkconfig does not exist yet; afterwards the existing sdkconfig
# wins and an edited defaults line is silently NOT applied until the sdkconfig
# is deleted by hand. That is exactly the "I changed the config and nothing
# happened" trap the whole hybrid move is meant to end.
#
# WHAT IT DOES: if the environment's defaults file
# (sdkconfig.<env>.defaults, next to platformio.ini) is newer than the
# generated .pio/build/<env>/sdkconfig, the generated file is deleted so that
# kconfgen rebuilds it from the defaults on this run. The defaults file is
# the only input, so this is always safe; it just costs the reconfigure.
#
# Only sdkconfig itself is touched — never the object files, so an edit here
# is a reconfigure plus the recompilation of what depends on sdkconfig.h, not
# a 25-minute full rebuild of the lib-builder tree.

import os

Import("env")

project_dir = env.subst("$PROJECT_DIR")
build_dir = env.subst("$BUILD_DIR")
pioenv = env["PIOENV"]

defaults = os.path.join(project_dir, "sdkconfig.%s.defaults" % pioenv)
generated = os.path.join(build_dir, "sdkconfig")

if os.path.isfile(defaults) and os.path.isfile(generated):
    if os.path.getmtime(defaults) > os.path.getmtime(generated):
        print("*** %s is newer than the generated sdkconfig: regenerating it from the defaults ***"
              % os.path.basename(defaults))
        for stale in (generated, generated + ".old"):
            try:
                os.remove(stale)
            except OSError:
                pass
