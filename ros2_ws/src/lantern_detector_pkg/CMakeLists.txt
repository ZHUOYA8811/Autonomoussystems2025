cmake_minimum_required(VERSION 3.8)
project(lantern_detector_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ---------- Component library ----------
add_library(lantern_detector_component SHARED
  src/lantern_detector_node.cpp
)

target_include_directories(lantern_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lantern_detector_component
  rclcpp rclcpp_components
  sensor_msgs geometry_msgs visualization_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  message_filters cv_bridge
)

# OpenCV is not an ament package -> link explicitly
target_link_libraries(lantern_detector_component ${OpenCV_LIBS})

rclcpp_components_register_nodes(lantern_detector_component
  "lantern_detector_pkg::LanternDetectorNode"
)

# ---------- Standalone executable ----------
add_executable(lantern_detector_node
  src/lantern_detector_main.cpp
)
target_link_libraries(lantern_detector_node lantern_detector_component)
ament_target_dependencies(lantern_detector_node rclcpp)

# ---------- Install ----------
install(TARGETS
  lantern_detector_component
  lantern_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()