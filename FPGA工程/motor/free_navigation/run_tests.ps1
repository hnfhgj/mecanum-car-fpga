$ErrorActionPreference = 'Stop'
$testRoot = $PSScriptRoot
$motorRoot = Split-Path -Parent $testRoot
$simRoot = Join-Path $testRoot 'test_work'
New-Item -ItemType Directory -Force -Path (Join-Path $simRoot 'src') | Out-Null
foreach ($rom in @('imu660ra_config.hex','dl1b_config.hex')) {
 Copy-Item -LiteralPath (Join-Path $motorRoot "src/$rom") -Destination (Join-Path $simRoot "src/$rom")
}
[xml]$project = (Get-Content -LiteralPath (Join-Path $motorRoot 'free_navigation.gprj') -Raw).Replace('version="1"','version="1.0"')
$rtl = @($project.Project.FileList.File | Where-Object { $_.enable -eq '1' -and $_.type -eq 'file.verilog' } | ForEach-Object { Join-Path $motorRoot $_.path })
$binRoot = 'E:/Xilinx/Vivado/2023.2/bin'
New-Item -ItemType Directory -Force -Path (Join-Path $simRoot 'square_nav') | Out-Null
foreach ($f in @('frame_fixture.txt','frame_live.txt','frame_missing.txt')) {
 Copy-Item -LiteralPath (Join-Path $testRoot $f) -Destination (Join-Path $simRoot "square_nav/$f")
}
Set-Location -LiteralPath $simRoot
& "$binRoot/xvlog.bat" --sv (Join-Path $testRoot 'top_tb.sv') (Join-Path $testRoot 'control_tb.v') (Join-Path $testRoot 'stream_tb.sv') (Join-Path $testRoot 'speed_tb.sv') (Join-Path $testRoot 'nav_tb.v') (Join-Path $testRoot 'manual_tb.sv') (Join-Path $testRoot 'snapshot_tb.sv') (Join-Path $testRoot 'cloud_stream_tb.sv') (Join-Path $testRoot 'hold_top_tb.sv') (Join-Path $testRoot 'frame_tb.v') @rtl *> (Join-Path $testRoot 'test_compile.log')
if ($LASTEXITCODE -ne 0) { throw 'RTL compile failed' }
foreach ($top in @('pc_control_tb','pc_bridge_tb','wireless_top_tb','wireless_stream_tb','speed_tb','basic_nav_tb','pc_motor_pipeline_tb','manual_tb','directional_tb','cloud_stream_tb','snapshot_tb','hold_top_tb','square_frame_tb')) {
 $snapshot = "free_97F4_$top"
 & "$binRoot/xelab.bat" $top -s $snapshot *> (Join-Path $testRoot "${top}_elaborate.log")
 if ($LASTEXITCODE -ne 0) { throw "Elaboration failed: $top" }
 & "$binRoot/xsim.bat" $snapshot -runall -log (Join-Path $testRoot "${top}.log") *> (Join-Path $testRoot "${top}_console.log")
 if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $top" }
 $log = Get-Content -LiteralPath (Join-Path $testRoot "${top}.log") -Raw
 if ($log -notmatch 'PASS ' -or $log -match 'Fatal:|ERROR:') { throw "Simulation did not pass: $top" }
 Write-Output ($log -split "`n" | Where-Object { $_ -match '^PASS ' })
}
