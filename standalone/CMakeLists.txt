cmake_minimum_required(VERSION 3.20)
project(dt46_aim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d dnn)
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATH_SUFFIXES eigen3 REQUIRED)
set(SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../src")
add_library(dt46_armor_model STATIC armor_model.cpp)
target_include_directories(dt46_armor_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dt46_armor_model PUBLIC ${OpenCV_LIBS})
set_target_properties(dt46_armor_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(dt46_aim SHARED
    aim.cpp
    ${SOURCE_ROOT}/rm_tracker/src/ekf.cpp
    ${SOURCE_ROOT}/rm_tracker/src/body_ekf.cpp
    ${SOURCE_ROOT}/rm_tracker/src/rm_tf_tools.cpp
    ${SOURCE_ROOT}/rm_tracker/src/tracker.cpp)
target_include_directories(dt46_aim PRIVATE
    ${SOURCE_ROOT}/rm_tracker/include
    ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dt46_aim PRIVATE dt46_armor_model ${OpenCV_LIBS})
set_target_properties(dt46_aim PROPERTIES PREFIX "")
if(WIN32)
    target_compile_definitions(dt46_aim PRIVATE _USE_MATH_DEFINES)
endif()
