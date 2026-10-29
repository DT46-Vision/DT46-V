param(
    [string]$EigenRoot = '',
    [string]$OutputDirectory = '',
    [string]$DelayConfig = ''
)
$ErrorActionPreference = 'Stop'
$visionRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if (!$EigenRoot) { $EigenRoot = Join-Path $visionRoot '../.deps/src/eigen-3.4.0' }
if (!$OutputDirectory) { $OutputDirectory = Join-Path $PSScriptRoot 'rerun' }
if (!$DelayConfig) { $DelayConfig = Join-Path $visionRoot 'tests/data/relative_aim_delays.cfg' }
$experimentBuild = Join-Path $visionRoot 'build/relative-aim-isolated'
cmake -S $visionRoot -B $experimentBuild -G Ninja -DDT46_RELATIVE_AIM_ONLY=ON "-DEIGEN3_INCLUDE_DIR=$EigenRoot" -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE) { throw 'configure failed' }
cmake --build $experimentBuild --parallel 2
if ($LASTEXITCODE) { throw 'build failed' }
ctest --test-dir $experimentBuild --output-on-failure
if ($LASTEXITCODE) { throw 'regression checks failed' }
& (Join-Path $experimentBuild 'dt46_relative_aim_check.exe') $OutputDirectory $DelayConfig
if ($LASTEXITCODE) { throw 'experiment checks failed; retain output for analysis' }
