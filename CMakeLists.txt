cmake_minimum_required(VERSION 3.8)
project(deepgis_vehicles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# MAVROS2 Vehicle Interface Node
add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)
ament_target_dependencies(vehicle_interface_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  mavros
  mavros_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts (note: telemetry/gps publishers now in deepgis_vehicles subpackage)
install(PROGRAMS 
  scripts/connect_pixhawk.sh
  scripts/test_gps_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install HydraSDR nodes
install(PROGRAMS 
  scripts/hydra_sdr/hydra_sdr_node.py
  scripts/hydra_sdr/spectrum_analyzer_node.py
  scripts/hydra_sdr/soapy_sdr_node.py
  scripts/hydra_sdr/sdr_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  mavros
  mavros_msgs
  tf2
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

