param(
    [string]$Python = 'python',
    [string]$OpenCVDir,
    [string]$EigenDir,
    [string[]]$RuntimeDir = @(),
    [string]$Generator = 'Ninja',
    [ValidateRange(1, 64)][int]$Jobs = 4,
    [string]$IndexUrl = 'https://pypi.tuna.tsinghua.edu.cn/simple'
)
$ErrorActionPreference = 'Stop'
if ($OpenCVDir -and !(Test-Path -LiteralPath $OpenCVDir -PathType Container)) {
    throw "OpenCVDir does not exist: $OpenCVDir"
}
if ($EigenDir -and !(Test-Path -LiteralPath (Join-Path $EigenDir 'Eigen/Core') -PathType Leaf)) {
    throw "EigenDir must contain Eigen/Core: $EigenDir"
}
$visionRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$environmentPython = Join-Path $visionRoot '.venv/Scripts/python.exe'
if (!(Test-Path -LiteralPath $environmentPython)) {
    & $Python -m venv (Join-Path $visionRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Python environment creation failed' }
}
$output = Join-Path $visionRoot ('outputs/setup_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Force -Path $output | Out-Null
& $environmentPython -m pip install --index-url $IndexUrl -e $PSScriptRoot *> (Join-Path $output 'pip.log')
if ($LASTEXITCODE -ne 0) { throw "Python dependency installation failed: $output/pip.log" }
if (!$OpenCVDir) { $OpenCVDir = Join-Path $visionRoot '.deps/native/opencv' }
if (!$EigenDir) { $EigenDir = Join-Path $visionRoot '.deps/native/deps/eigen-3.4.0' }
if (!(Test-Path -LiteralPath $OpenCVDir) -or !(Test-Path -LiteralPath $EigenDir)) {
    & cmake ('-DWORK_DIR=' + (Join-Path $visionRoot '.deps/native')) ('-DJOBS=' + $Jobs) -DDOWNLOAD_REGION=cn -P (Join-Path $visionRoot 'cmake/bootstrap.cmake') *> (Join-Path $output 'native-dependencies.log')
    if ($LASTEXITCODE -ne 0) { throw "Native dependency preparation failed: $output/native-dependencies.log" }
}
$buildArguments = @('--opencv-dir', $OpenCVDir, '--eigen-dir', $EigenDir, '--generator', $Generator, '--jobs', $Jobs)
foreach ($directory in $RuntimeDir) { $buildArguments += @('--runtime-dir', $directory) }
& (Join-Path $visionRoot '.venv/Scripts/dt46-build.exe') @buildArguments *> (Join-Path $output 'core-build.log')
if ($LASTEXITCODE -ne 0) { throw "Native core build failed: $output/core-build.log" }
Write-Output "DT46 environment prepared. Logs: $output. Run the Windows acceptance checks before treating this as an accepted integration."
