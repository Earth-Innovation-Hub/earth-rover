cmake_minimum_required(VERSION 3.8)
project(rtlsdr_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RtlsdrSignal.msg"
  "msg/RtlsdrSpectrum.msg"
  "msg/GpsData.msg"
  DEPENDENCIES std_msgs
)

# Install Python package manually (to avoid conflicts with ament_python_install_package)
# Get Python version
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Install Python package directory
install(DIRECTORY
  rtlsdr_ros2/
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/rtlsdr_ros2
  USE_SOURCE_PERMISSIONS
)

# Install Python executables (wrappers that call the modules)
install(PROGRAMS
  rtlsdr_ros2/rtlsdr_reader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rtlsdr_reader
)
install(PROGRAMS
  rtlsdr_ros2/spectrum_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spectrum_visualizer
)
install(PROGRAMS
  rtlsdr_ros2/signal_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME signal_recorder
)
install(PROGRAMS
  rtlsdr_ros2/gps_reader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_reader
)
install(PROGRAMS
  rtlsdr_ros2/gps_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_monitor
)
install(PROGRAMS
  rtlsdr_ros2/gps_position_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_position_publisher
)
install(PROGRAMS
  rtlsdr_ros2/gps_map_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_map_visualizer
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

