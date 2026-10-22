cmake_minimum_required(VERSION 3.8)
project(rm_tracker)

# 1. 显式指定 C++17 标准 (ROS 2 Humble 官方推荐规范)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 开启 O3 极致优化与硬件本地向量化加速，使 Eigen 矩阵运算性能最大化
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rm_interfaces REQUIRED)

# 查找外部依赖高性能库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# 编译核心算法库 (tracker_core)
# ==============================================================================
add_library(tracker_core
  src/ekf.cpp
  src/rm_tf_tools.cpp
  src/tracker.cpp
)

# 绑定头文件路径
target_include_directories(tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# 链接 Eigen 和 OpenCV
target_link_libraries(tracker_core ${OpenCV_LIBS} Eigen3::Eigen)

# 给核心库挂载 ROS 消息依赖，防止 cpp 内部 #include 接口时报错
ament_target_dependencies(tracker_core
  rclcpp
  sensor_msgs
  geometry_msgs
  rm_interfaces
)

# ==============================================================================
# 编译 ROS 2 节点可执行文件 (对齐修改为 rm_tracker_node)
# ==============================================================================
# 核心修改点：编译目标正式改名为 rm_tracker_node
add_executable(rm_tracker_node src/node.cpp)

target_link_libraries(rm_tracker_node tracker_core)

ament_target_dependencies(rm_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  rm_interfaces
)

# ==============================================================================
# 安装与部署规则
# ==============================================================================
# 核心修改点：安装的可执行文件目标对齐修改为 rm_tracker_node
install(TARGETS rm_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件与独立启动 launch 文件夹
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
