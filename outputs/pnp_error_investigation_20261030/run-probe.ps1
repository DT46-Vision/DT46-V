$ErrorActionPreference='Stop'
$repo=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $repo
try {
    . ../.local/runtime-path-check/activate.ps1
    $opencv=Join-Path (Split-Path $repo -Parent) '.deps/environment-path-check/install/runtime-path-check/opencv'
    $objects=Join-Path $repo 'build/aim-validation-full/CMakeFiles/dt46_pnp_check.dir/src/rm_detector/src'
    $libs=@('dnn','calib3d','videoio','features2d','flann','imgcodecs','imgproc','core') | ForEach-Object {Join-Path $opencv "x64/mingw/lib/libopencv_$($_)4120.dll.a"}
    & 'D:/msys2/ucrt64/bin/g++.exe' -O2 -std=c++20 -DNOMINMAX -D_USE_MATH_DEFINES "-I$repo/src/rm_detector/include" "-I$repo/../.deps/src/json/include" "-I$opencv/include" "$PSScriptRoot/probe-pnp.cpp" "$objects/pnp.cpp.obj" "$objects/detector.cpp.obj" "$objects/classifier.cpp.obj" @libs -o "$PSScriptRoot/probe-pnp.exe"
    if($LASTEXITCODE){throw 'Probe compilation failed'}
    $env:PATH=(Join-Path $repo 'build/aim-validation-full/standalone')+';'+$env:PATH
    & "$PSScriptRoot/probe-pnp.exe" "$PSScriptRoot/pose-probes.jsonl" "$PSScriptRoot/probe-results.jsonl"
    if($LASTEXITCODE){throw 'Probe execution failed'}
    if(Test-Path "$PSScriptRoot/mesh-probes.jsonl") {
        & "$PSScriptRoot/probe-pnp.exe" "$PSScriptRoot/mesh-probes.jsonl" "$PSScriptRoot/mesh-probe-results.jsonl"
        if($LASTEXITCODE){throw 'Mesh probe execution failed'}
    }
} finally {Pop-Location}
