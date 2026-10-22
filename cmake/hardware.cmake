add_library(dt46_hardware project/modules/hardware/protocol.cpp project/modules/hardware/serial_port.cpp)
target_include_directories(dt46_hardware PUBLIC project)
target_compile_definitions(dt46_hardware PUBLIC NOMINMAX)
