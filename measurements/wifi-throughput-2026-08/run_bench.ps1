param(
    [string]$Ip    = '192.168.1.81',
    [string]$Plan  = 'Bs100,600,100,15',
    [int]   $Masodperc = 0,          # 0 = a Plan alapjan szamolja
    [string]$Cimke = 'meres',
    [string]$Com   = 'COM10'
)
$ErrorActionPreference = 'Stop'
$proj = 'C:\Users\RF\Documents\PlatformIO\Projects\FG23-SDR-ESP32-S3-streaming'
$src = (Get-ChildItem 'G:\*\Python_Measurements\WSPR\RF_measurements\wifi_sink.py').FullName
Copy-Item $src (Join-Path $proj 'wifi_sink.py') -Force

# idotartam becslese a rampabol: lepcsok * (dwell + warm 2s) + tartalek
if ($Masodperc -le 0) {
    if ($Plan -match '^Bs(\d+),(\d+),(\d+),(\d+)') {
        $n = [math]::Floor(([int]$Matches[2] - [int]$Matches[1]) / [int]$Matches[3]) + 1
        $Masodperc = $n * ([int]$Matches[4] + 2) + 15
    } else { $Masodperc = 90 }
}

$stamp   = Get-Date -Format 'MMdd_HHmm'
$espLog  = Join-Path $proj ($Cimke + '_' + $stamp + '_esp.log')
$sinkLog = Join-Path $proj ($Cimke + '_' + $stamp + '_sink.log')
$csv     = Join-Path $proj ($Cimke + '_' + $stamp + '.csv')

Write-Output ("cel: $Ip   terv: $Plan   ido: $Masodperc s   cimke: $Cimke")

$p = New-Object System.IO.Ports.SerialPort $Com,115200,'None',8,'One'
$p.DtrEnable = $false
$p.RtsEnable = $false
$p.NewLine = "`n"
$p.Open()
Start-Sleep -Milliseconds 400
$p.WriteLine('B0')
Start-Sleep -Milliseconds 800
[void]$p.ReadExisting()
$p.WriteLine($Plan)
Write-Output "-> $Plan elkuldve"

Start-Sleep -Milliseconds 800
$proc = Start-Process -FilePath 'python' `
    -ArgumentList @('wifi_sink.py',$Ip,'--csv',$csv,'--window','2') `
    -WorkingDirectory $proj -RedirectStandardOutput $sinkLog `
    -RedirectStandardError (Join-Path $proj 'bench_sink.err') `
    -NoNewWindow -PassThru
Write-Output ('-> nyelo elindult, PID ' + $proc.Id)

$sb = New-Object System.Text.StringBuilder
$vege = (Get-Date).AddSeconds($Masodperc)
while ((Get-Date) -lt $vege) {
    Start-Sleep -Milliseconds 400
    $n = $p.BytesToRead
    if ($n -gt 0) {
        $b = New-Object byte[] $n
        [void]$p.Read($b, 0, $n)
        [void]$sb.Append([System.Text.Encoding]::ASCII.GetString($b))
    }
    if ($proc.HasExited -and ((Get-Date).AddSeconds(5) -lt $vege)) { }
}
$p.WriteLine('B0')
Start-Sleep -Seconds 1
$p.Close()
$sb.ToString() | Out-File -FilePath $espLog -Encoding utf8

$w = 0
while ((-not $proc.HasExited) -and ($w -lt 25)) { Start-Sleep -Seconds 1; $w++ }
if (-not $proc.HasExited) { $proc.Kill() }

Write-Output '=== ESP ==='
(Select-String -Path $espLog -Pattern 'BENCH-STEP|BENCH-END|nincs adat').Line
Write-Output '=== NYELO ==='
Get-Content $sinkLog -Tail 30
Write-Output '=== NYELO HIBA ==='
Get-Content (Join-Path $proj 'bench_sink.err') -Tail 8
Write-Output ''
Write-Output ("fajlok: " + $csv)
