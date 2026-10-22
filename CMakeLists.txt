cmake_minimum_required(VERSION 3.24)
project(dt46_vision VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CTest)
include(FetchContent)

if(NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json 3.11 QUIET)
    if(NOT TARGET nlohmann_json::nlohmann_json)
        FetchContent_Declare(json
            URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
            URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
            TIMEOUT 120 DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        FetchContent_MakeAvailable(json)
    endif()
endif()

# Existing native installations can be supplied through normal CMake cache paths.
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d dnn)
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3 REQUIRED)
add_subdirectory(standalone)

add_library(dt46_pipeline project/modules/vision/pipeline.cpp)
target_include_directories(dt46_pipeline PUBLIC project standalone ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dt46_pipeline PUBLIC dt46_aim nlohmann_json::nlohmann_json ${OpenCV_LIBS})
target_compile_definitions(dt46_pipeline PUBLIC NOMINMAX)
if(BUILD_TESTING)
    add_executable(dt46_pipeline_check tests/pipeline_check.cpp)
    target_link_libraries(dt46_pipeline_check PRIVATE dt46_pipeline)
    add_test(NAME native_pipeline COMMAND dt46_pipeline_check "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
