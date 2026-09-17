# Contributing

Thanks for your interest. A few ground rules that keep this project legally clean and technically honest.

**Licensing.** By contributing you agree your contribution is licensed under the licence of the directory it lands in (MIT for firmware/host, GPL-3.0 for the SDR++ module, CERN-OHL-P-2.0 for hardware, CC-BY-4.0 for docs and data). Do not copy code from GPL projects into MIT directories — LightAPRS, LibAPRS, ZeroAPRS and similar are inspiration only.

**Vendor code.** Silicon Labs RAIL is used through its public API and headers. Never disassemble or reverse-engineer `librail_*.a`; PHY-level findings must come from public documentation, the Radio Configurator, or your own measurements.

**Claims need measurements.** If a change affects RF performance, timing, or throughput, add or update the corresponding capture/script under `measurements/` and cite it in the PR. "Works for me" is not a result.

**Secrets.** WiFi credentials, API keys and callsign-specific configuration go in files listed in `.gitignore`, never in commits.

**Style.** Firmware is C (C11) with the existing CLI-command conventions (single-letter diagnostic commands are documented in `docs/cli.md`). Host code is C for `libionos`, C++ for the SDR++ module. Keep PRs small and describe what you measured.
