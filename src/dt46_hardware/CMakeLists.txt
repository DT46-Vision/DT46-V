cmake_minimum_required(VERSION 3.16)
project(dt46_hardware)
find_package(ament_cmake REQUIRED)
set(DT46_PROJECT "${CMAKE_CURRENT_SOURCE_DIR}/../../project")
add_library(dt46_hardware STATIC
  "${DT46_PROJECT}/modules/hardware/protocol.cpp"
  "${DT46_PROJECT}/modules/hardware/serial_port.cpp")
target_compile_features(dt46_hardware PUBLIC cxx_std_20)
target_include_directories(dt46_hardware PUBLIC
  $<BUILD_INTERFACE:${DT46_PROJECT}> $<INSTALL_INTERFACE:include>)
target_compile_definitions(dt46_hardware PUBLIC NOMINMAX)
install(TARGETS dt46_hardware EXPORT export_dt46_hardware ARCHIVE DESTINATION lib INCLUDES DESTINATION include)
install(DIRECTORY "${DT46_PROJECT}/modules/hardware" DESTINATION include/modules FILES_MATCHING PATTERN "*.hpp")
ament_export_targets(export_dt46_hardware HAS_LIBRARY_TARGET)
ament_package()
