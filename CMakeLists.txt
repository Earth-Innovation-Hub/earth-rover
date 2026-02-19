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

# Install scripts
install(PROGRAMS 
  scripts/connect_pixhawk.sh
  scripts/test_gps_publisher.py
  scripts/deepgis_telemetry_publisher.py
  scripts/adsb_aircraft_state_vectors_node.py
  scripts/adsb_state_vectors_to_rviz_node.py
  scripts/adsb_state_vectors_plot_2d.py
  scripts/adsb_state_vectors_plot_glide.py
  scripts/landmark_vo_plot_2d.py
  scripts/landmark_vo_plot_fisheye.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared SDR/ADS-B module (used by both HydraSDR and RTL-SDR decoders)
install(PROGRAMS
  scripts/sdr_adsb_common.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install HydraSDR nodes
install(PROGRAMS 
  scripts/hydra_sdr/hydra_sdr_node.py
  scripts/hydra_sdr/spectrum_analyzer_node.py
  scripts/hydra_sdr/sdr_visualizer.py
  scripts/hydra_sdr/adsb_decoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install RTL-SDR V4 nodes
install(PROGRAMS 
  scripts/rtl_sdr/rtl_sdr_node.py
  scripts/rtl_sdr/rtl_adsb_decoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (only if include/ directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

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

