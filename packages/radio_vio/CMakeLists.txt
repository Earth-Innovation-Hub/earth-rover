cmake_minimum_required(VERSION 3.8)
project(radio_vio)

find_package(ament_cmake REQUIRED)

# Top-level Python nodes
install(PROGRAMS
  scripts/adsb_aircraft_state_vectors_node.py
  scripts/adsb_state_vectors_to_rviz_node.py
  scripts/adsb_state_vectors_plot_2d.py
  scripts/adsb_state_vectors_plot_glide.py
  scripts/landmark_vo_plot_2d.py
  scripts/landmark_vo_plot_fisheye.py
  scripts/sdr_adsb_common.py
  DESTINATION lib/${PROJECT_NAME}
)

# HydraSDR / shared IQ nodes (flat install: imports sdr_adsb_common from lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/hydra_sdr/hydra_sdr_node.py
  scripts/hydra_sdr/spectrum_analyzer_node.py
  scripts/hydra_sdr/sdr_visualizer.py
  scripts/hydra_sdr/adsb_decoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# RTL-SDR V4 nodes
install(PROGRAMS
  scripts/rtl_sdr/rtl_sdr_node.py
  scripts/rtl_sdr/rtl_adsb_decoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Hardware setup helpers (optional)
install(PROGRAMS
  scripts/rtl_sdr/install_rtl_sdr.sh
  scripts/hydra_sdr/install_hydra_sdr.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
