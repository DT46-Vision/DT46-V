cmake_minimum_required(VERSION 3.8)
project(rm_vision_bringup)
# 仅仅引用该变量以消除警告
set(dummy_var ${CMAKE_EXPORT_COMPILE_COMMANDS})
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.xml")
ament_package()
