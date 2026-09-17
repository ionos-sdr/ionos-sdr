# sdrpp_build_all.ps1 - FULL build of SDR++ + fg23_scan_source on Windows, in one command.
# HA7DCD 2026-08-15.
#
# USAGE (PowerShell, administrator NOT required; on one line):
#   cd C:\utils
#   Set-ExecutionPolicy -Scope Process Bypass -Force
#   .\sdrpp_build_all.ps1 -Zip C:\utils\fg23_scan_source.zip
#
# PREREQUISITES (the script CHECKS these but does not install them):
#   - Visual Studio 2022 (Community is sufficient) with the "Desktop development with C++" workload
#   - Git for Windows
#   - vcpkg:  git clone https://github.com/microsoft/vcpkg C:\vcpkg ; C:\vcpkg\bootstrap-vcpkg.bat
#   - PothosSDR installed EXACTLY at: C:\Program Files\PothosSDR   (required for volk.dll)
#   - (optional) RtAudio: C:\Program Files (x86)\RtAudio  - without it -NoAudio (automatic)
#       Building RtAudio (ADMINISTRATOR PowerShell, because of Program Files), on one line:
#       cd C:\utils; git clone https://github.com/thestk/rtaudio; cd rtaudio; cmake -B build -A x64 -DRTAUDIO_BUILD_TESTING=OFF "-DCMAKE_INSTALL_PREFIX=C:\Program Files (x86)\RtAudio"; cmake --build build --config Release --target install
#       (for cmake, give the full path to the VS-bundled cmake.exe if it is not on the PATH)
#
# RESULT: C:\utils\sdrpp_fg23\  - portable folder (sdrpp.exe + DLLs + modules\),
#         can be copied to a laptop and run there without building.

param(
  [string]$Zip     = "C:\utils\fg23_scan_source.zip",
  [string]$Src     = "C:\utils\SDRPlusPlus",
  [string]$Vcpkg   = "C:\vcpkg",
  [string]$Out     = "C:\utils\sdrpp_fg23",
  [switch]$NoAudio,          # if RtAudio is not installed
  [switch]$SkipVcpkg,        # if fftw3/glfw3/zstd are already installed
  [switch]$WithHackRF,       # also build the HackRF source (the lib comes from PothosSDR)
  [string]$Generator = ""    # CMake generator override, e.g. "Visual Studio 18 2026"
)

$ErrorActionPreference = "Stop"
function Step($s) { Write-Host "`n=== $s" -ForegroundColor Cyan }
function Fail($s) { Write-Host "`nERROR: $s" -ForegroundColor Red; exit 1 }

# ---------- 0. prerequisites ----------
Step "Prerequisites"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { Fail "Visual Studio Installer (vswhere.exe) not found. Install VS 2022 Community with the C++ desktop workload." }
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { Fail "VS is present but the C++ toolset (MSVC) is NOT installed. VS Installer -> Modify -> 'Desktop development with C++'." }
Write-Host "VS: $vsPath"
$vsVer = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property catalog_productLineVersion
$vsMajor = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion).Split(".")[0]
# The generator is derived from the ACTUAL major version: 16=2019 (v142), 17=2022 (v143), 18=2026 (v180)
$gen = switch ($vsMajor) { "16" { "Visual Studio 16 2019" } "17" { "Visual Studio 17 2022" } "18" { "Visual Studio 18 2026" } default { "Visual Studio $vsMajor $vsVer" } }
if ($Generator) { $gen = $Generator }
Write-Host "VS version: $vsVer (major $vsMajor)  ->  CMake generator: $gen"

if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Fail "git not found on the PATH (Git for Windows)." }
$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if (-not $cmake) {
  $vsCmake = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
  if (Test-Path $vsCmake) { $env:PATH = (Split-Path $vsCmake) + ";" + $env:PATH; Write-Host "cmake from VS: $vsCmake" }
  else { Fail "cmake not found (neither on the PATH nor in VS). Install it from cmake.org or add the VS 'C++ CMake tools' component." }
}
if (-not (Test-Path (Join-Path $Vcpkg "vcpkg.exe"))) { Fail "vcpkg.exe not found at: $Vcpkg  (git clone https://github.com/microsoft/vcpkg $Vcpkg ; $Vcpkg\bootstrap-vcpkg.bat)" }
if (-not (Test-Path "C:\Program Files\PothosSDR\bin\volk.dll")) { Fail "PothosSDR not found under C:\Program Files\PothosSDR (the core requires volk.dll)." }
$haveRt = Test-Path "C:\Program Files (x86)\RtAudio"
if (-not $haveRt -and -not $NoAudio) { Write-Host "WARNING: RtAudio not found -> audio_sink disabled (-NoAudio applied automatically)" -ForegroundColor Yellow; $NoAudio = $true }
if (-not (Test-Path $Zip)) { Fail "Module zip not found: $Zip" }

# ---------- 1. vcpkg packages ----------
if (-not $SkipVcpkg) {
  Step "vcpkg: fftw3 glfw3 zstd (x64-windows) - 10-20 minutes on first run"
  & (Join-Path $Vcpkg "vcpkg.exe") install fftw3:x64-windows glfw3:x64-windows zstd:x64-windows
  if ($LASTEXITCODE -ne 0) { Fail "vcpkg install failed" }
}

# ---------- 2. source + module ----------
Step "SDR++ source"
if (-not (Test-Path $Src)) { git clone https://github.com/AlexandreRouma/SDRPlusPlus.git $Src } else { Write-Host "already present: $Src" }
$dst = Join-Path $Src "source_modules\fg23_scan_source"
$tmp = Join-Path $env:TEMP "fg23_scan_unzip"
if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
Expand-Archive -Path $Zip -DestinationPath $tmp
if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
Copy-Item (Join-Path $tmp "fg23_scan_source") $dst -Recurse
if (Test-Path (Join-Path $dst "esp32")) { Remove-Item (Join-Path $dst "esp32") -Recurse -Force }
Write-Host "module: $dst"

$cm = Join-Path $Src "CMakeLists.txt"
$txt = Get-Content $cm -Raw
if ($txt -notmatch "OPT_BUILD_FG23_SCAN_SOURCE") {
  $txt = $txt -replace '(option\(OPT_BUILD_SPYSERVER_SOURCE[^\r\n]*\r?\n)',
    ('$1' + 'option(OPT_BUILD_FG23_SCAN_SOURCE "Build FG23 Scan Source Module (no dependencies required)" ON)' + "`r`n")
  $txt = $txt -replace '(endif \(OPT_BUILD_SPYSERVER_SOURCE\)\r?\n)',
    ('$1' + "`r`nif (OPT_BUILD_FG23_SCAN_SOURCE)`r`nadd_subdirectory(`"source_modules/fg23_scan_source`")`r`nendif (OPT_BUILD_FG23_SCAN_SOURCE)`r`n")
  Set-Content -Path $cm -Value $txt -NoNewline
  Write-Host "CMakeLists.txt patched"
} else { Write-Host "CMakeLists.txt already contains the module" }

# ---------- 3. cmake + build ----------
Step "CMake configure"
$bld = Join-Path $Src "build"
New-Item -ItemType Directory -Force -Path $bld | Out-Null
# discard an old cache made with a different generator (otherwise cmake will not switch generators)
if (Test-Path (Join-Path $bld "CMakeCache.txt")) { Remove-Item (Join-Path $bld "CMakeCache.txt") -Force }
if (Test-Path (Join-Path $bld "CMakeFiles")) { Remove-Item (Join-Path $bld "CMakeFiles") -Recurse -Force }
$opts = @(
  "-DOPT_BUILD_AIRSPY_SOURCE=OFF", "-DOPT_BUILD_AIRSPYHF_SOURCE=OFF",
  "-DOPT_BUILD_PLUTOSDR_SOURCE=OFF", "-DOPT_BUILD_RTL_SDR_SOURCE=OFF",
  "-DOPT_BUILD_M17_DECODER=OFF", "-DOPT_BUILD_AUDIO_SOURCE=OFF",
  "-DOPT_BUILD_FG23_SCAN_SOURCE=ON"
)
$opts += if ($WithHackRF) { "-DOPT_BUILD_HACKRF_SOURCE=ON" } else { "-DOPT_BUILD_HACKRF_SOURCE=OFF" }
if ($NoAudio) { $opts += "-DOPT_BUILD_AUDIO_SINK=OFF" }
$tc = Join-Path $Vcpkg "scripts\buildsystems\vcpkg.cmake"
Push-Location $bld
& cmake .. "-DCMAKE_TOOLCHAIN_FILE=$tc" -G $gen -A x64 @opts
if ($LASTEXITCODE -ne 0) { Pop-Location; Fail "cmake configure failed (see above)" }
Step "Build (Release) - 10-30 minutes"
& cmake --build . --config Release -- /m
if ($LASTEXITCODE -ne 0) { Pop-Location; Fail "build failed (see above)" }
Pop-Location

# ---------- 4. portable package ----------
Step "Portable folder: $Out"
if (Test-Path $Out) { Remove-Item $Out -Recurse -Force }
New-Item -ItemType Directory -Force -Path (Join-Path $Out "modules") | Out-Null
Copy-Item (Join-Path $Src "root\*") $Out -Recurse -Force            # res/, bandplans etc.
Copy-Item (Join-Path $bld "Release\*") $Out -Force                   # sdrpp.exe, sdrpp_core.dll, vcpkg DLLs
Copy-Item "C:\Program Files\PothosSDR\bin\volk.dll" $Out -Force
if ($WithHackRF) { Copy-Item "C:\Program Files\PothosSDR\bin\hackrf.dll" $Out -Force -ErrorAction SilentlyContinue }
Get-ChildItem $bld -Recurse -Filter "*.dll" | Where-Object {
  $_.FullName -match "\\Release\\" -and $_.Directory.FullName -ne (Join-Path $bld "Release") -and $_.Name -notmatch "^sdrpp_core"
} | ForEach-Object { Copy-Item $_.FullName (Join-Path $Out "modules") -Force }
# the modules\ folder must contain module DLLs only; dependencies (e.g. rtaudio) go to the root
foreach ($f in @("rtaudio.dll","rtaudiod.dll")) {
  $p = Join-Path $Out "modules\$f"; if (Test-Path $p) { Move-Item $p $Out -Force }
}
# RtAudio runtime DLL next to audio_sink (if RtAudio is installed)
$rtdll = "C:\Program Files (x86)\RtAudio\bin\rtaudio.dll"
if (-not $NoAudio -and (Test-Path $rtdll)) { Copy-Item $rtdll $Out -Force; Write-Host "rtaudio.dll copied (audio_sink)" }

Write-Host ""
Write-Host "DONE." -ForegroundColor Green
Write-Host "  Launch:  $Out\sdrpp.exe   (modules are loaded from the modules\ folder)"
Write-Host "  The folder can be copied to a laptop; there add it in the Module Manager: fg23_scan_source"
Write-Host "  Source: 'FG23 Scan' -> Mode SCAN -> Span/Bins -> Play"
$m = Join-Path $Out "modules\fg23_scan_source.dll"
if (Test-Path $m) { Write-Host "  fg23_scan_source.dll: OK" -ForegroundColor Green } else { Write-Host "  fg23_scan_source.dll MISSING - check the build output!" -ForegroundColor Red }
