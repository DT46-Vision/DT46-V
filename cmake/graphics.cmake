set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip
    URL_HASH SHA256=b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE TIMEOUT 180)
FetchContent_MakeAvailable(glfw)
add_library(dt46_debug
    project/modules/vision/debug_images.cpp
    project/modules/vision/debug_window.cpp
    project/platform/debug_control.cpp)
target_include_directories(dt46_debug PRIVATE "${glfw_SOURCE_DIR}/deps")
target_link_libraries(dt46_debug PUBLIC dt46_pipeline PRIVATE glfw)
target_compile_definitions(dt46_debug PRIVATE GLFW_INCLUDE_NONE)
if(WIN32)
    target_link_libraries(dt46_debug PRIVATE comdlg32)
endif()
