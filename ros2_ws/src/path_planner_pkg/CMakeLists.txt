cmake_minimum_required(VERSION 3.16)
project(path_planner_pkg)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 查找依赖包
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(builtin_interfaces REQUIRED)

# 依赖state_machine包（用于Command/Answer消息）
find_package(state_machine REQUIRED)

# ============================================================
# 定义可执行文件
# ============================================================
add_executable(path_planner_node
  src/path_planner_node.cpp
  src/occupancy_map.cpp
  src/astar_planner.cpp
  src/rrt_explorer.cpp
)

# 头文件路径
target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# 链接依赖
ament_target_dependencies(path_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_ros
  builtin_interfaces
  state_machine
)

# ============================================================
# 安装
# ============================================================
install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
