# sdrpp_prepare.ps1 - Prepare the SDR++ source tree + fg23_scan_source module on Windows
# HA7DCD 2026-08-15.  Run from PowerShell, e.g. under C:\utils\:
#   cd C:\utils
#   .\sdrpp_prepare.ps1 -Zip "C:\Users\RF\Documents\PlatformIO\Projects\FG23-SDR-ESP32-S3-streaming\fg23_scan_source.zip"
#
# What it does: clones SDR++ (if not present), copies the module under
# source_modules, and adds the OPT_BUILD_FG23_SCAN_SOURCE option + add_subdirectory
# to the root CMakeLists.txt (modelled on the SPYSERVER block). It does NOT build;
# the cmake commands are printed at the end.

param(
  [string]$Zip = "$PSScriptRoot\fg23_scan_source.zip",
  [string]$Dir = "$PWD\SDRPlusPlus"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $Zip)) { throw "Zip not found: $Zip" }

if (-not (Test-Path $Dir)) {
  Write-Host "== git clone SDR++ -> $Dir"
  git clone https://github.com/AlexandreRouma/SDRPlusPlus.git $Dir
} else {
  Write-Host "== SDR++ already present: $Dir (not cloning again)"
}

$mods = Join-Path $Dir "source_modules"
Write-Host "== extracting module -> $mods\fg23_scan_source"
$tmp = Join-Path $env:TEMP "fg23_scan_unzip"
if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
Expand-Archive -Path $Zip -DestinationPath $tmp
$dst = Join-Path $mods "fg23_scan_source"
if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
Copy-Item (Join-Path $tmp "fg23_scan_source") $dst -Recurse
# the esp32/ subfolder is not needed for the SDR++ build
if (Test-Path (Join-Path $dst "esp32")) { Remove-Item (Join-Path $dst "esp32") -Recurse -Force }

$cm = Join-Path $Dir "CMakeLists.txt"
$txt = Get-Content $cm -Raw
if ($txt -notmatch "OPT_BUILD_FG23_SCAN_SOURCE") {
  Write-Host "== CMakeLists.txt patch"
  $txt = $txt -replace '(option\(OPT_BUILD_SPYSERVER_SOURCE[^\r\n]*\r?\n)',
    ('$1' + 'option(OPT_BUILD_FG23_SCAN_SOURCE "Build FG23 Scan Source Module (no dependencies required)" ON)' + "`r`n")
  $txt = $txt -replace '(endif \(OPT_BUILD_SPYSERVER_SOURCE\)\r?\n)',
    ('$1' + "`r`nif (OPT_BUILD_FG23_SCAN_SOURCE)`r`nadd_subdirectory(`"source_modules/fg23_scan_source`")`r`nendif (OPT_BUILD_FG23_SCAN_SOURCE)`r`n")
  Set-Content -Path $cm -Value $txt -NoNewline
} else {
  Write-Host "== CMakeLists.txt already contains the module"
}

Write-Host ""
Write-Host "DONE. Next steps (per the readme; Visual Studio 2022 + vcpkg + PothosSDR + RtAudio required):"
Write-Host "  cd $Dir"
Write-Host "  mkdir build; cd build"
Write-Host '  cmake .. "-DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake" -G "Visual Studio 17 2022" -A x64 `'
Write-Host '    -DOPT_BUILD_AIRSPY_SOURCE=OFF -DOPT_BUILD_AIRSPYHF_SOURCE=OFF -DOPT_BUILD_HACKRF_SOURCE=OFF `'
Write-Host '    -DOPT_BUILD_PLUTOSDR_SOURCE=OFF -DOPT_BUILD_RTL_SDR_SOURCE=OFF -DOPT_BUILD_M17_DECODER=OFF'
Write-Host "  cmake --build . --config Release"
Write-Host "The module: build\source_modules\fg23_scan_source\Release\fg23_scan_source.dll"
