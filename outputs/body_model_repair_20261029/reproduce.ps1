param([string]$OpenCVRoot='', [string]$EigenRoot='', [string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$visionRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
if(!$OpenCVRoot){$OpenCVRoot=Join-Path $visionRoot '../.deps/environment-path-check/install/runtime-path-check/opencv'}
if(!$EigenRoot){$EigenRoot=Join-Path $visionRoot '../.deps/src/eigen-3.4.0'}
if(!$OutputDirectory){$OutputDirectory=Join-Path $PSScriptRoot 'rerun'}
New-Item -ItemType Directory -Force $OutputDirectory | Out-Null
$OutputDirectory=(Resolve-Path $OutputDirectory).Path
$experimentBuild=Join-Path $visionRoot 'build/body-repair-native'
$jsonRoot=Join-Path $visionRoot '../.deps/src/json'
cmake -S $visionRoot -B $experimentBuild -G Ninja -DCMAKE_BUILD_TYPE=Release -DDT46_HEADLESS_TESTS=ON "-DOpenCV_DIR=$OpenCVRoot" "-DEIGEN3_INCLUDE_DIR=$EigenRoot" "-DFETCHCONTENT_SOURCE_DIR_JSON=$jsonRoot"
if($LASTEXITCODE){throw 'configure failed'}
cmake --build $experimentBuild --parallel 2
if($LASTEXITCODE){throw 'build failed'}
$previousPath=$env:PATH
try {
    $env:PATH=(Join-Path $OpenCVRoot 'x64/mingw/bin')+';'+(Join-Path $experimentBuild 'standalone')+';'+$env:PATH
    ctest --test-dir $experimentBuild --output-on-failure
    if($LASTEXITCODE){throw 'native checks failed'}
    & (Join-Path $experimentBuild 'dt46_body_ekf_check.exe') (Join-Path $OutputDirectory 'body')
    if($LASTEXITCODE){throw 'body checks failed'}
    & (Join-Path $experimentBuild 'dt46_tracker_model_check.exe') (Join-Path $OutputDirectory 'tracker')
    if($LASTEXITCODE){throw 'tracker checks failed'}
    $baselineRoot=Join-Path $PSScriptRoot 'baseline_source'
    $sourcePaths=@('src/tracker.cpp','src/ekf.cpp','src/rm_tf_tools.cpp','include/rm_tracker/tracker.hpp','include/rm_tracker/ekf.hpp','include/rm_tracker/rm_tf_tools.hpp')
    foreach($sourcePath in $sourcePaths){
        $target=Join-Path $baselineRoot $sourcePath
        New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
        $content=git -C $visionRoot show ('6c3e6f81fd6e193b9318961da5e140cbc9064074:src/rm_tracker/'+$sourcePath)
        if($LASTEXITCODE){throw 'baseline extraction failed'}
        [System.IO.File]::WriteAllLines($target,$content,[System.Text.UTF8Encoding]::new($false))
    }
    $cmakeSource=@'
cmake_minimum_required(VERSION 3.24)
project(tracker_reference LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
add_executable(tracker_reference "${VISION_ROOT}/tests/tracker_model_check.cpp" src/tracker.cpp src/ekf.cpp src/rm_tf_tools.cpp)
target_include_directories(tracker_reference PRIVATE include "${EIGEN3_INCLUDE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(tracker_reference PRIVATE _USE_MATH_DEFINES NOMINMAX)
target_link_libraries(tracker_reference PRIVATE ${OpenCV_LIBS})
'@
    [System.IO.File]::WriteAllText((Join-Path $baselineRoot 'CMakeLists.txt'),$cmakeSource,[System.Text.UTF8Encoding]::new($false))
    $baselineBuild=Join-Path $visionRoot 'build/body-repair-baseline'
    cmake -S $baselineRoot -B $baselineBuild -G Ninja -DCMAKE_BUILD_TYPE=Release "-DVISION_ROOT=$visionRoot" "-DEIGEN3_INCLUDE_DIR=$EigenRoot" "-DOpenCV_DIR=$OpenCVRoot"
    if($LASTEXITCODE){throw 'baseline configure failed'}
    cmake --build $baselineBuild --parallel 2
    if($LASTEXITCODE){throw 'baseline build failed'}
    & (Join-Path $baselineBuild 'tracker_reference.exe') (Join-Path $OutputDirectory 'baseline') --reference
    if($LASTEXITCODE){throw 'baseline execution failed'}
    if((Get-FileHash (Join-Path $OutputDirectory 'tracker/inputs.csv')).Hash -ne (Get-FileHash (Join-Path $OutputDirectory 'baseline/inputs.csv')).Hash){throw 'comparison inputs differ'}
} finally { $env:PATH=$previousPath }
