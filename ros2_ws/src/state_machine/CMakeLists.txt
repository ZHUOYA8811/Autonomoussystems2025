cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# 1. find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# 必须查找这个包来生成自定义消息
find_package(rosidl_default_generators REQUIRED)

# 2. 生成自定义消息接口 (Command.msg 和 Answer.msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/Answer.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# 3. 定义可执行文件 (Node)
add_executable(state_machine_node 
  src/state_machine.cpp
)

# 4. 指定头文件路径 (让编译器找到 statemachine.hpp)
target_include_directories(state_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. 链接依赖
# 链接标准的 ROS 2 库
ament_target_dependencies(state_machine_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# 链接本项目生成的自定义消息接口 (这是 ROS 2 的标准写法)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(state_machine_node "${cpp_typesupport_target}")

# 6. 安装 (用于 colcon install)
install(TARGETS
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 7. 测试与打包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
