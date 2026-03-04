cmake_minimum_required(VERSION 3.8)
project(waypoint_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(state_machine REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)

# Locate glog / nlopt that ship with mav_trajectory_generation
get_filename_component(
  MAV_TRAJECTORY_GENERATION_PREFIX
  "${mav_trajectory_generation_DIR}/../../.."
  ABSOLUTE
)
find_library(
  GLOG_LIB
  NAMES glog
  PATHS "${MAV_TRAJECTORY_GENERATION_PREFIX}/lib"
  NO_DEFAULT_PATH
)
find_library(
  NLOPT_LIB
  NAMES nlopt
  PATHS "${MAV_TRAJECTORY_GENERATION_PREFIX}/lib"
  NO_DEFAULT_PATH
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(waypoint_node src/waypoint_node.cpp)

ament_target_dependencies(waypoint_node
  rclcpp
  nav_msgs
  geometry_msgs
  trajectory_msgs
  state_machine
  mav_trajectory_generation
  mav_msgs
  mav_planning_msgs
)

target_link_directories(waypoint_node
  PRIVATE
    "${MAV_TRAJECTORY_GENERATION_PREFIX}/lib"
)

target_link_libraries(waypoint_node
  Eigen3::Eigen
)

if(GLOG_LIB)
  target_link_libraries(waypoint_node ${GLOG_LIB})
endif()

if(NLOPT_LIB)
  target_link_libraries(waypoint_node ${NLOPT_LIB})
endif()

# Install targets
install(TARGETS
  waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
