cmake_minimum_required(VERSION 3.24)
# Standalone dependency preparation: cmake -DWORK_DIR=<directory> -DJOBS=4 -P cmake/bootstrap.cmake
if(NOT DEFINED WORK_DIR)
    get_filename_component(WORK_DIR "${CMAKE_CURRENT_LIST_DIR}/../.cache/native" ABSOLUTE)
endif()
get_filename_component(WORK_DIR "${WORK_DIR}" ABSOLUTE)
if(NOT DEFINED JOBS)
    set(JOBS 4)
endif()
file(MAKE_DIRECTORY "${WORK_DIR}/deps")

function(archive name url digest)
    set(path "${WORK_DIR}/deps/${name}.tar.gz")
    if(NOT EXISTS "${path}")
        file(DOWNLOAD "${url}" "${path}.part" STATUS status TIMEOUT 300 TLS_VERIFY ON)
        list(GET status 0 code)
        if(NOT code EQUAL 0)
            message(FATAL_ERROR "Download ${name}: ${status}")
        endif()
        file(SHA256 "${path}.part" actual)
        if(NOT actual STREQUAL digest)
            message(FATAL_ERROR "SHA256 mismatch: ${name}")
        endif()
        file(RENAME "${path}.part" "${path}")
    endif()
    file(SHA256 "${path}" actual)
    if(NOT actual STREQUAL digest)
        message(FATAL_ERROR "SHA256 mismatch: ${path}")
    endif()
    if(NOT EXISTS "${WORK_DIR}/deps/${name}/CMakeLists.txt")
        file(ARCHIVE_EXTRACT INPUT "${path}" DESTINATION "${WORK_DIR}/deps")
    endif()
endfunction()

archive(opencv-4.12.0 https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.12.0
    44c106d5bb47efec04e531fd93008b3fcd1d27138985c5baf4eafac0e1ec9e9d)
archive(eigen-3.4.0 https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72)

function(ffmpeg_file name blob digest)
    set(directory "${WORK_DIR}/opencv-build/3rdparty/ffmpeg")
    file(MAKE_DIRECTORY "${directory}")
    set(path "${directory}/${name}")
    if(EXISTS "${path}")
        file(MD5 "${path}" actual)
        if(actual STREQUAL digest)
            return()
        endif()
    endif()
    # GitHub's API is also reachable on networks that cannot resolve raw.githubusercontent.com.
    file(DOWNLOAD "https://api.github.com/repos/opencv/opencv_3rdparty/git/blobs/${blob}"
        "${path}.part" HTTPHEADER "Accept: application/vnd.github.raw+json"
        STATUS status TIMEOUT 300 TLS_VERIFY ON)
    list(GET status 0 code)
    if(NOT code EQUAL 0)
        message(FATAL_ERROR "FFmpeg download ${name}: ${status}")
    endif()
    file(MD5 "${path}.part" actual)
    if(NOT actual STREQUAL digest)
        message(FATAL_ERROR "OpenCV-pinned FFmpeg checksum mismatch: ${name}")
    endif()
    file(RENAME "${path}.part" "${path}")
endfunction()
if(CMAKE_HOST_WIN32)
    ffmpeg_file(opencv_videoio_ffmpeg_64.dll 1f21d9816452c8df5a8c5cae13a952b7ff7a5853 e5c6936240201064b15bcecf1816e8f4)
    ffmpeg_file(opencv_videoio_ffmpeg.dll 271d69e440d0a83484f372428d1970f08714d78d 2821ea672a11147a70974d760a54e9bc)
    ffmpeg_file(ffmpeg_version.cmake 77b6b0b0ad4e435580df4e2ae838a70f043364b8 8862c87496e2e8c375965e1277dee1c7)
endif()

set(opencv_download_options)
if(DOWNLOAD_REGION STREQUAL "cn")
    list(APPEND opencv_download_options -DOPENCV_DOWNLOAD_MIRROR_ID=gitcode)
endif()
execute_process(COMMAND "${CMAKE_COMMAND}"
    -S "${WORK_DIR}/deps/opencv-4.12.0" -B "${WORK_DIR}/opencv-build" -G Ninja
    -DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=${WORK_DIR}/opencv"
    -DBUILD_LIST=core,imgproc,calib3d,dnn,imgcodecs,videoio -DBUILD_SHARED_LIBS=ON ${opencv_download_options}
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF
    -DOPENCV_PYTHON_SKIP_DETECTION=ON -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF
    -DBUILD_JAVA=OFF -DWITH_IPP=OFF -DWITH_OPENCL=OFF -DWITH_ITT=OFF
    -DWITH_FFMPEG=ON -DWITH_MSMF=OFF -DWITH_OPENEXR=OFF -DBUILD_PROTOBUF=ON -DCPU_DISPATCH=
    COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${WORK_DIR}/opencv-build"
    --target install --parallel "${JOBS}" COMMAND_ERROR_IS_FATAL ANY)
message(STATUS "Dependencies ready. Configure with -DOpenCV_DIR=${WORK_DIR}/opencv -DEIGEN3_INCLUDE_DIR=${WORK_DIR}/deps/eigen-3.4.0")
