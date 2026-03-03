cmake_minimum_required(VERSION 3.8)
project(planner_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(state_machine REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common kdtree filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Octomap
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

# Find mav_trajectory_generation
find_package(mav_trajectory_generation REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(explorer_node
  src/explorer_node.cpp
)

add_executable(trajectory_planner_node
  src/simple_trajectory_planner.cpp
)

# Dependencies for explorer_node
ament_target_dependencies(explorer_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  state_machine
  pcl_conversions
  pcl_ros
  octomap_msgs
  tf2
  tf2_ros
)

# Dependencies for trajectory_planner_node
ament_target_dependencies(trajectory_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  state_machine
  mav_planning_msgs
  mav_trajectory_generation
  Eigen3
)

# Link libraries
target_link_libraries(explorer_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Install targets
install(TARGETS
  explorer_node
  trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
