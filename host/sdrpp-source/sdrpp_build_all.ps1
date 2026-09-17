# sdrpp_build_all.ps1 - SDR++ + fg23_scan_source TELJES build Windowson, egy parancsbol.
# HA7DCD 2026-08-15.
#
# HASZNALAT (PowerShell, NEM kell rendszergazda; egy sorban):
#   cd C:\utils
#   Set-ExecutionPolicy -Scope Process Bypass -Force
#   .\sdrpp_build_all.ps1 -Zip C:\utils\fg23_scan_source.zip
#
# ELOFELTETELEK (ezeket a script ELLENORZI, de nem telepiti):
#   - Visual Studio 2022 (Community jo) a "Desktop development with C++" workloaddal
#   - Git for Windows
#   - vcpkg:  git clone https://github.com/microsoft/vcpkg C:\vcpkg ; C:\vcpkg\bootstrap-vcpkg.bat
#   - PothosSDR telepitve PONTOSAN ide: C:\Program Files\PothosSDR   (volk.dll miatt kell)
#   - (opcionalis) RtAudio: C:\Program Files (x86)\RtAudio  - enelkul -NoAudio (automatikus)
#       RtAudio forditasa (RENDSZERGAZDAI PowerShell, a Program Files miatt), egy sorban:
#       cd C:\utils; git clone https://github.com/thestk/rtaudio; cd rtaudio; cmake -B build -A x64 -DRTAUDIO_BUILD_TESTING=OFF "-DCMAKE_INSTALL_PREFIX=C:\Program Files (x86)\RtAudio"; cmake --build build --config Release --target install
#       (a cmake-hez a VS-es cmake.exe-t add meg teljes uttal, ha nincs a PATH-on)
#
# EREDMENY: C:\utils\sdrpp_fg23\  - hordozhato mappa (sdrpp.exe + DLL-ek + modules\),
#           atmasolhato a laptopra, ott futtathato forditas nelkul.

param(
  [string]$Zip     = "C:\utils\fg23_scan_source.zip",
  [string]$Src     = "C:\utils\SDRPlusPlus",
  [string]$Vcpkg   = "C:\vcpkg",
  [string]$Out     = "C:\utils\sdrpp_fg23",
  [switch]$NoAudio,          # ha nincs RtAudio telepitve
  [switch]$SkipVcpkg,        # ha az fftw3/glfw3/zstd mar fent van
  [switch]$WithHackRF,       # HackRF source is (PothosSDR-bol jon a lib)
  [string]$Generator = ""    # CMake generator felulirasa, pl. "Visual Studio 18 2026"
)

$ErrorActionPreference = "Stop"
function Step($s) { Write-Host "`n=== $s" -ForegroundColor Cyan }
function Fail($s) { Write-Host "`nHIBA: $s" -ForegroundColor Red; exit 1 }

# ---------- 0. elofeltetelek ----------
Step "Elofeltetelek"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { Fail "Nincs Visual Studio Installer (vswhere.exe). Telepitsd a VS 2022 Community-t a C++ desktop workloaddal." }
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { Fail "Van VS, de NINCS benne a C++ toolset (MSVC). VS Installer -> Modify -> 'Desktop development with C++'." }
Write-Host "VS: $vsPath"
$vsVer = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property catalog_productLineVersion
$vsMajor = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion).Split(".")[0]
# A generator a TENYLEGES fo verziobol: 16=2019 (v142), 17=2022 (v143), 18=2026 (v180)
$gen = switch ($vsMajor) { "16" { "Visual Studio 16 2019" } "17" { "Visual Studio 17 2022" } "18" { "Visual Studio 18 2026" } default { "Visual Studio $vsMajor $vsVer" } }
if ($Generator) { $gen = $Generator }
Write-Host "VS verzio: $vsVer (major $vsMajor)  ->  CMake generator: $gen"

if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Fail "Nincs git a PATH-on (Git for Windows)." }
$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if (-not $cmake) {
  $vsCmake = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
  if (Test-Path $vsCmake) { $env:PATH = (Split-Path $vsCmake) + ";" + $env:PATH; Write-Host "cmake a VS-bol: $vsCmake" }
  else { Fail "Nincs cmake (sem PATH-on, sem a VS-ben). Telepitsd a cmake.org-rol vagy a VS 'C++ CMake tools' komponenst." }
}
if (-not (Test-Path (Join-Path $Vcpkg "vcpkg.exe"))) { Fail "Nincs vcpkg.exe itt: $Vcpkg  (git clone https://github.com/microsoft/vcpkg $Vcpkg ; $Vcpkg\bootstrap-vcpkg.bat)" }
if (-not (Test-Path "C:\Program Files\PothosSDR\bin\volk.dll")) { Fail "Nincs PothosSDR a C:\Program Files\PothosSDR alatt (volk.dll kell a core-nak)." }
$haveRt = Test-Path "C:\Program Files (x86)\RtAudio"
if (-not $haveRt -and -not $NoAudio) { Write-Host "FIGYELEM: nincs RtAudio -> audio_sink kikapcsolva (-NoAudio automatikusan)" -ForegroundColor Yellow; $NoAudio = $true }
if (-not (Test-Path $Zip)) { Fail "Nincs meg a modul zip: $Zip" }

# ---------- 1. vcpkg csomagok ----------
if (-not $SkipVcpkg) {
  Step "vcpkg: fftw3 glfw3 zstd (x64-windows) - elso alkalommal 10-20 perc"
  & (Join-Path $Vcpkg "vcpkg.exe") install fftw3:x64-windows glfw3:x64-windows zstd:x64-windows
  if ($LASTEXITCODE -ne 0) { Fail "vcpkg install sikertelen" }
}

# ---------- 2. forras + modul ----------
Step "SDR++ forras"
if (-not (Test-Path $Src)) { git clone https://github.com/AlexandreRouma/SDRPlusPlus.git $Src } else { Write-Host "mar megvan: $Src" }
$dst = Join-Path $Src "source_modules\fg23_scan_source"
$tmp = Join-Path $env:TEMP "fg23_scan_unzip"
if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
Expand-Archive -Path $Zip -DestinationPath $tmp
if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
Copy-Item (Join-Path $tmp "fg23_scan_source") $dst -Recurse
if (Test-Path (Join-Path $dst "esp32")) { Remove-Item (Join-Path $dst "esp32") -Recurse -Force }
Write-Host "modul: $dst"

$cm = Join-Path $Src "CMakeLists.txt"
$txt = Get-Content $cm -Raw
if ($txt -notmatch "OPT_BUILD_FG23_SCAN_SOURCE") {
  $txt = $txt -replace '(option\(OPT_BUILD_SPYSERVER_SOURCE[^\r\n]*\r?\n)',
    ('$1' + 'option(OPT_BUILD_FG23_SCAN_SOURCE "Build FG23 Scan Source Module (no dependencies required)" ON)' + "`r`n")
  $txt = $txt -replace '(endif \(OPT_BUILD_SPYSERVER_SOURCE\)\r?\n)',
    ('$1' + "`r`nif (OPT_BUILD_FG23_SCAN_SOURCE)`r`nadd_subdirectory(`"source_modules/fg23_scan_source`")`r`nendif (OPT_BUILD_FG23_SCAN_SOURCE)`r`n")
  Set-Content -Path $cm -Value $txt -NoNewline
  Write-Host "CMakeLists.txt patchelve"
} else { Write-Host "CMakeLists.txt mar tartalmazza a modult" }

# ---------- 3. cmake + build ----------
Step "CMake configure"
$bld = Join-Path $Src "build"
New-Item -ItemType Directory -Force -Path $bld | Out-Null
# regi, mas generatorral keszult cache eldobasa (kulonben a cmake nem valt generatort)
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
if ($LASTEXITCODE -ne 0) { Pop-Location; Fail "cmake configure sikertelen (lasd fent)" }
Step "Build (Release) - 10-30 perc"
& cmake --build . --config Release -- /m
if ($LASTEXITCODE -ne 0) { Pop-Location; Fail "build sikertelen (lasd fent)" }
Pop-Location

# ---------- 4. hordozhato csomag ----------
Step "Hordozhato mappa: $Out"
if (Test-Path $Out) { Remove-Item $Out -Recurse -Force }
New-Item -ItemType Directory -Force -Path (Join-Path $Out "modules") | Out-Null
Copy-Item (Join-Path $Src "root\*") $Out -Recurse -Force            # res/, bandplans stb.
Copy-Item (Join-Path $bld "Release\*") $Out -Force                   # sdrpp.exe, sdrpp_core.dll, vcpkg dll-ek
Copy-Item "C:\Program Files\PothosSDR\bin\volk.dll" $Out -Force
if ($WithHackRF) { Copy-Item "C:\Program Files\PothosSDR\bin\hackrf.dll" $Out -Force -ErrorAction SilentlyContinue }
Get-ChildItem $bld -Recurse -Filter "*.dll" | Where-Object {
  $_.FullName -match "\\Release\\" -and $_.Directory.FullName -ne (Join-Path $bld "Release") -and $_.Name -notmatch "^sdrpp_core"
} | ForEach-Object { Copy-Item $_.FullName (Join-Path $Out "modules") -Force }
# a modules\ mappaban csak modul-DLL-ek legyenek; a fuggosegek (pl. rtaudio) a gyokerbe
foreach ($f in @("rtaudio.dll","rtaudiod.dll")) {
  $p = Join-Path $Out "modules\$f"; if (Test-Path $p) { Move-Item $p $Out -Force }
}
# RtAudio runtime DLL az audio_sink melle (ha van RtAudio telepitve)
$rtdll = "C:\Program Files (x86)\RtAudio\bin\rtaudio.dll"
if (-not $NoAudio -and (Test-Path $rtdll)) { Copy-Item $rtdll $Out -Force; Write-Host "rtaudio.dll bemasolva (audio_sink)" }

Write-Host ""
Write-Host "KESZ." -ForegroundColor Green
Write-Host "  Inditas:  $Out\sdrpp.exe   (a modulok a modules\ mappabol toltodnek)"
Write-Host "  A mappa atmasolhato a laptopra; ott a Module Manager-ben add hozza: fg23_scan_source"
Write-Host "  Source: 'FG23 Scan' -> Mode SCAN -> Span/Bins -> Play"
$m = Join-Path $Out "modules\fg23_scan_source.dll"
if (Test-Path $m) { Write-Host "  fg23_scan_source.dll: OK" -ForegroundColor Green } else { Write-Host "  fg23_scan_source.dll HIANYZIK - nezd a build kimenetet!" -ForegroundColor Red }
