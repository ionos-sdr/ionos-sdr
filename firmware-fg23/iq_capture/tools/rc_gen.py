#!/usr/bin/env python3
"""rc_gen.py — headless Multi-PHY radio configurator: radio_settings.radioconf -> rail_config.c/.h

    Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT

  Runs the SAME configurator Studio runs (the SDK's copy, so the version
  matches the project: simplicity_sdk/platform/radio/efr32_multiphy_configurator)
  with Studio's bundled Python 3.10 and its numpy/scipy (ext-site-packages).
  On the unchanged radioconf the output is byte-identical to Studio's autogen.

  Setup once (Windows, PowerShell):
    robocopy <SDK>\\platform\\radio\\efr32_multiphy_configurator rcmp /E /XD tests __pycache__
    mkdir rcmp\\studio_pymath\\ext-site-packages
    cmd /c mklink /J rcmp\\studio_pymath\\ext-site-packages\\win ^
        C:\\SiliconLabs\\SimplicityStudio\\v5\\developer\\adapter_packs\\python\\ext-site-packages
  (set_pymath_paths.py looks for studio_pymath/ext-site-packages/win next to the package.)

  usage:
    C:\\SiliconLabs\\SimplicityStudio\\v5\\developer\\adapter_packs\\python\\python.exe -W ignore ^
        rc_gen.py <rcmp dir> config\\rail\\radio_settings.radioconf <outdir>
  then copy <outdir>\\rail_config.c/.h into the project's autogen\\ and build.

  Override values in the radioconf: enums by their plain member name
  (FSK2, Gaussian, LSB_FIRST, FIXED_LENGTH), booleans True/False.
"""
import sys, os
sys.path.insert(0, os.path.abspath(sys.argv[1]))
from efr32multiphyconfig import Efr32MultiPHYConfigurator
from pylib_multi_phy_model.multi_phy_configuration_model.rail_scripts_wrapper import RAILScriptsWrapper

out = Efr32MultiPHYConfigurator().configure(multi_phy_config_model=open(sys.argv[2], encoding='utf-8').read())
m = out['multi_phy_config_model_memory_object']
print('status_code', m.status_code)
if int(m.status_code) != 0:
    sys.exit('configurator error - see the messages above')
os.makedirs(sys.argv[3], exist_ok=True)
RAILScriptsWrapper.dump_output_files(m, sys.argv[3])
