# Earth Rover  --  developer convenience targets.
# These shortcuts collapse the repeated build/source/launch sequences
# observed in bash history into single make targets.
#
# Quick start:
#   make help           - list all targets
#   make landmark-vo    - the dominant inner-loop (build + source + launch)
#   make trike-up       - start the full trike stack
#   make vcs-up         - start the VCS triplet (rosbridge + web_video_server + Django)
#
# Override:
#   ROS_DISTRO=humble  ROS2_WS=$$HOME/ros2_ws  PKG=deepgis_vehicles

ROS_DISTRO       ?= humble
ROS2_WS          ?= $(HOME)/ros2_ws
PKG              ?= deepgis_vehicles
EARTH_ROVER_HOME ?= $(CURDIR)
STARTUP_DIR      ?= $(EARTH_ROVER_HOME)/scripts/startup

PIXHAWK_DEVICE   ?= /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0
PIXHAWK_BAUD     ?= 921600
GCS_URL          ?= udp://192.168.1.7:14550

# Compose a single shell line that activates ROS env, then runs $(CMD).
# Used so each recipe runs in one shell with the workspace sourced.
WITH_ROS = bash -c 'set -e; \
    source /opt/ros/$(ROS_DISTRO)/setup.bash; \
    [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash; \
    $(CMD)'

.PHONY: help build rebuild build-all source \
        landmark-vo landmark-vo-fisheye rtl-adsb adsb-state adsb-glide sdr \
        mavros mavros-bg \
        gh-cam meta-cam velo rs-cam spinnaker-left spinnaker-right \
        rosbridge web-video vcs-runserver \
        vcs-up vcs-down vcs-status \
        trike-up trike-down trike-status trike-minimal \
        units-install units-list units-tail \
        mission-up mission-down mission-status mission-tail \
        ui-up ui-down ui-status \
        archive-now archive-status archive-tail archive-cancel \
        info

help:
	@printf "Earth Rover make targets\n"
	@printf "========================\n"
	@printf "  build              - colcon build --packages-select $(PKG) (+ source)\n"
	@printf "  build-all          - colcon build everything (+ source)\n"
	@printf "  rebuild            - rm -rf build/ install/ log/ && colcon build\n"
	@printf "\n"
	@printf "  landmark-vo        - build + source + landmark_vo_plot_2d.launch.py\n"
	@printf "  landmark-vo-fisheye - build + source + landmark_vo_plot_fisheye.launch.py\n"
	@printf "  rtl-adsb           - build + source + rtl_adsb.launch.py\n"
	@printf "  adsb-state         - adsb_aircraft_state_vectors.launch.py\n"
	@printf "  adsb-glide         - adsb_state_vectors_plot_glide.launch.py\n"
	@printf "  sdr                - sdr.launch.py\n"
	@printf "\n"
	@printf "  mavros             - mavros px4.launch (FCU+GCS from PIXHAWK_*/GCS_URL vars)\n"
	@printf "  mavros-bg          - same, in background via systemd --user\n"
	@printf "  gh-cam             - launch_grasshopper.sh\n"
	@printf "  meta-cam           - metavision_driver\n"
	@printf "  velo               - velodyne VLP16\n"
	@printf "  rs-cam             - realsense2_camera\n"
	@printf "  spinnaker-{left,right} - Spinnaker camera launches\n"
	@printf "\n"
	@printf "  rosbridge          - rosbridge_websocket\n"
	@printf "  web-video          - web_video_server\n"
	@printf "  vcs-runserver      - Django dev server (runserver 0.0.0.0:8000)\n"
	@printf "  vcs-up / vcs-down / vcs-status - bring the VCS triplet up/down\n"
	@printf "\n"
	@printf "  trike-up / trike-down / trike-status - full trike stack\n"
	@printf "  trike-minimal      - minimal: mavros + rosbridge + web_video_server\n"
	@printf "\n"
	@printf "  units-install      - install ~/.config/systemd/user/er-*.service units\n"
	@printf "  units-list         - systemctl --user list-units 'er-*'\n"
	@printf "  units-tail UNIT=x  - journalctl --user -u er-x -f\n"
	@printf "\n"
	@printf "Manual mission launch (NOTHING auto-starts at boot anymore):\n"
	@printf "  mission-up         - start the whole er-mission.target graph\n"
	@printf "  mission-down       - stop everything\n"
	@printf "  mission-status     - one-line state for every er-* unit\n"
	@printf "  mission-tail UNIT=x - journalctl --user -u er-x -f (alias of units-tail)\n"
	@printf "  ui-up / ui-down    - just the web-frontend layer\n"
	@printf "                       (er-vcs + er-rosbridge + er-web-video + er-kiosk)\n"
	@printf "  archive-now        - one-shot SSD->NAS rsync (no daily timer)\n"
	@printf "  archive-cancel     - stop a running archive\n"
	@printf "  archive-tail       - tail the archive log\n"
	@printf "\n"
	@printf "  info               - print resolved variables\n"

info:
	@echo "ROS_DISTRO       = $(ROS_DISTRO)"
	@echo "ROS2_WS          = $(ROS2_WS)"
	@echo "PKG              = $(PKG)"
	@echo "EARTH_ROVER_HOME = $(EARTH_ROVER_HOME)"
	@echo "PIXHAWK_DEVICE   = $(PIXHAWK_DEVICE)"
	@echo "PIXHAWK_BAUD     = $(PIXHAWK_BAUD)"
	@echo "GCS_URL          = $(GCS_URL)"

source:
	@echo "Run:  source /opt/ros/$(ROS_DISTRO)/setup.bash && source $(ROS2_WS)/install/setup.bash"

build:
	cd $(ROS2_WS) && $(MAKE) -f /dev/null _colcon_select PKG=$(PKG)

# Internal helper recipes that actually exec colcon - keeps the WITH_ROS pattern simple.
_colcon_select:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(PKG) && source install/setup.bash'

build-all:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build && source install/setup.bash'

rebuild:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && rm -rf build/ install/ log/ && colcon build'

# ---- The dominant inner-loop launches ---------------------------------------

landmark-vo:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(PKG) && source install/setup.bash && exec ros2 launch $(PKG) landmark_vo_plot_2d.launch.py estimated_position_topic:=/adsb/rtl_adsb_decoder_node/estimated_position'

landmark-vo-fisheye:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(PKG) && source install/setup.bash && exec ros2 launch $(PKG) landmark_vo_plot_fisheye.launch.py'

rtl-adsb:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(PKG) && source install/setup.bash && exec ros2 launch $(PKG) rtl_adsb.launch.py'

adsb-state:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(PKG) adsb_aircraft_state_vectors.launch.py'

adsb-glide:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(PKG) adsb_state_vectors_plot_glide.launch.py'

sdr:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(PKG) sdr.launch.py'

# ---- MAVROS -----------------------------------------------------------------

mavros:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash; \
		exec ros2 launch mavros px4.launch fcu_url:="$(PIXHAWK_DEVICE):$(PIXHAWK_BAUD)" gcs_url:="$(GCS_URL)"'

mavros-bg:
	systemctl --user start er-mavros.service
	@echo "Started er-mavros.service.  Logs: journalctl --user -u er-mavros -f"

# ---- Cameras / sensors -------------------------------------------------------

gh-cam:
	cd $(ROS2_WS) && exec ./launch_grasshopper.sh

meta-cam:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch metavision_driver driver_node.launch.py'

velo:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py'

rs-cam:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch realsense2_camera rs_launch.py'

spinnaker-left:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch spinnaker_camera_driver driver_node.launch.py camera_name:=left'

spinnaker-right:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch spinnaker_camera_driver driver_node.launch.py serial:=22312674 camera_name:=right'

# ---- VCS triplet -------------------------------------------------------------

rosbridge:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'

web-video:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 run web_video_server web_video_server'

vcs-runserver:
	cd $(EARTH_ROVER_HOME)/vehicle_control_station && exec python3 manage.py runserver 0.0.0.0:8000

vcs-up:
	$(STARTUP_DIR)/vcs_up.sh

vcs-down:
	$(STARTUP_DIR)/vcs_down.sh

vcs-status:
	$(STARTUP_DIR)/vcs_status.sh

# ---- Trike stack -------------------------------------------------------------

trike-up:
	$(STARTUP_DIR)/run_trike_stack.sh

trike-down:
	$(STARTUP_DIR)/stop_trike_stack.sh

trike-status:
	$(STARTUP_DIR)/check_status.sh

trike-minimal:
	$(STARTUP_DIR)/run_minimal_stack.sh

# ---- systemd user units ------------------------------------------------------

units-install:
	$(STARTUP_DIR)/install_user_units.sh

units-list:
	systemctl --user list-units 'er-*' --all

units-tail:
ifndef UNIT
	$(error UNIT is unset.  Use:  make units-tail UNIT=mavros)
endif
	journalctl --user -u er-$(UNIT) -f

# ---- Manual mission launch ---------------------------------------------------
# Auto-start at boot was deliberately removed.  These targets are the supported
# way to bring the rover up; each invocation is an explicit operator decision.

# UI layer the operator usually wants up first so the Django mission console
# at http://localhost:8000/mission/ can act as a launch panel for everything
# else.  Order matters: rosbridge + web_video before vcs (Django depends on
# both), kiosk last (waits for vcs).
UI_UNITS = er-rosbridge.service er-web-video.service er-vcs.service er-kiosk.service

mission-up:
	systemctl --user start er-mission.target
	@echo
	@echo "Mission target started.  Watch services come up with:"
	@echo "    make mission-status"

mission-down:
	systemctl --user stop er-mission.target
	@sleep 2
	systemctl --user reset-failed 'er-*' 2>/dev/null || true
	@echo "Mission target stopped."

mission-status:
	@systemctl --user list-units 'er-*' --all --no-legend \
	    | awk '{ printf "  %-32s %-8s %-12s %s\n", $$1, $$3, $$4, substr($$0, index($$0,$$5)) }'

mission-tail:
ifndef UNIT
	$(error UNIT is unset.  Use:  make mission-tail UNIT=mavros)
endif
	journalctl --user -u er-$(UNIT) -f

ui-up:
	systemctl --user start $(UI_UNITS)
	@echo
	@echo "Web-frontend layer started:"
	@echo "  Django dashboard : http://localhost:8000/"
	@echo "  Mission console  : http://localhost:8000/mission/"
	@echo "  rosbridge        : ws://localhost:9090/"
	@echo "  web_video        : http://localhost:8080/"

ui-down:
	systemctl --user stop $(UI_UNITS)
	@echo "Web-frontend layer stopped."

ui-status:
	@for u in $(UI_UNITS); do \
	    state=$$(systemctl --user is-active $$u); \
	    printf "  %-28s %s\n" "$$u" "$$state"; \
	done

# ---- One-shot SSD->NAS archive rsync ----------------------------------------
# The .timer that used to fire this on boot+5min and daily 03:00 was disabled.
# Run it explicitly when you want to archive yesterday-and-earlier files.

archive-now:
	systemctl --user start --no-block er-rsync-archive.service
	@echo "Archive started in the background.  Tail with:  make archive-tail"

archive-status:
	@systemctl --user status --no-pager er-rsync-archive.service | head -15 || true

archive-tail:
	tail -F $(HOME)/ssd-xtreme-transfer/rsync-archive.log

archive-cancel:
	systemctl --user stop er-rsync-archive.service
	systemctl --user reset-failed er-rsync-archive.service 2>/dev/null || true
	@echo "Archive cancelled."
