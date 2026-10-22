cmake_minimum_required(VERSION 3.20)
project(dt46_aim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d dnn)
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3 REQUIRED)
set(SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../src")
add_library(dt46_aim SHARED
    aim.cpp
    ${SOURCE_ROOT}/rm_detector/src/detector.cpp
    ${SOURCE_ROOT}/rm_detector/src/classifier.cpp
    ${SOURCE_ROOT}/rm_detector/src/pnp.cpp
    ${SOURCE_ROOT}/rm_tracker/src/ekf.cpp
    ${SOURCE_ROOT}/rm_tracker/src/rm_tf_tools.cpp
    ${SOURCE_ROOT}/rm_tracker/src/tracker.cpp)
target_include_directories(dt46_aim PRIVATE
    ${SOURCE_ROOT}/rm_detector/include ${SOURCE_ROOT}/rm_tracker/include
    ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dt46_aim PRIVATE ${OpenCV_LIBS})
set_target_properties(dt46_aim PROPERTIES PREFIX "")
if(WIN32)
    target_compile_definitions(dt46_aim PRIVATE _USE_MATH_DEFINES)
endif()
include(CTest)
if(BUILD_TESTING)
    add_executable(dt46_core_check core_check.cpp
        ${SOURCE_ROOT}/rm_detector/src/detector.cpp
        ${SOURCE_ROOT}/rm_detector/src/classifier.cpp
        ${SOURCE_ROOT}/rm_detector/src/pnp.cpp
        ${SOURCE_ROOT}/rm_tracker/src/rm_tf_tools.cpp)
    target_include_directories(dt46_core_check PRIVATE
        ${SOURCE_ROOT}/rm_detector/include ${SOURCE_ROOT}/rm_tracker/include
        ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(dt46_core_check PRIVATE ${OpenCV_LIBS})
    if(WIN32)
        target_compile_definitions(dt46_core_check PRIVATE _USE_MATH_DEFINES)
    endif()
    add_test(NAME calibrated_pnp_and_optical_rotation COMMAND dt46_core_check)
endif()
