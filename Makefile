# Earth Rover  --  developer convenience targets.
# These shortcuts collapse the repeated build/source/launch sequences
# observed in bash history into single make targets.
#
# Quick start:
#   make help           - list all targets
#   make landmark-vo    - the dominant inner-loop (build + source + launch)
#   make vcs-up         - start the VCS triplet (rosbridge + web_video_server + Django)
#
# Override:
#   ROS_DISTRO=jazzy  ROS2_WS=$$HOME/ros2_ws  PKG=deepgis_vehicles

ROS_DISTRO       ?= jazzy
ROS2_WS          ?= $(HOME)/ros2_ws
PKG              ?= deepgis_vehicles
RADIO_PKG        ?= radio_vio
EARTH_ROVER_HOME ?= $(CURDIR)
# colcon does not see packages/ when invoked from ~/ros2_ws unless symlinked; this path fixes that.
RADIO_PKG_SRC    ?= $(EARTH_ROVER_HOME)/packages/$(RADIO_PKG)
STARTUP_DIR      ?= $(EARTH_ROVER_HOME)/scripts/startup

PIXHAWK_DEVICE   ?= /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0
PIXHAWK_BAUD     ?= 921600
GCS_URL          ?= udp://@192.168.0.6:14550

# Compose a single shell line that activates ROS env, then runs $(CMD).
# Used so each recipe runs in one shell with the workspace sourced.
WITH_ROS = bash -c 'set -e; \
    source /opt/ros/$(ROS_DISTRO)/setup.bash; \
    [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash; \
    $(CMD)'

.PHONY: help build build-radio rebuild build-all source \
        landmark-vo landmark-vo-fisheye radio-stack rtl-adsb adsb-state adsb-glide sdr \
        system-launch mavros mavros-bg \
        gh-cam-left gh-cam-right meta-cam velo rs-cam spinnaker-left spinnaker-right \
        record-bag record-bag-mavros record-bag-stereo \
        rosbridge web-video vcs-runserver \
        vcs-up vcs-down vcs-status \
        units-install units-list units-tail \
        mission-up mission-down mission-status mission-tail \
        ui-up ui-down ui-status \
        archive-now archive-status archive-tail archive-cancel \
        hotkeys-install hotkeys-uninstall hotkeys-status \
        hotkey-trike-start hotkey-trike-stop hotkey-bag-start hotkey-bag-stop \
        hotkey-tail-trike hotkey-tail-bag \
        info

help:
	@printf "Earth Rover make targets\n"
	@printf "========================\n"
	@printf "  build              - colcon build --packages-select $(PKG) (+ source)\n"
	@printf "  build-radio        - colcon build --packages-select $(RADIO_PKG)\n"
	@printf "  build-all          - colcon build everything (+ source)\n"
	@printf "  rebuild            - rm -rf build/ install/ log/ && colcon build\n"
	@printf "\n"
	@printf "  landmark-vo        - build $(RADIO_PKG) + source + landmark_vo_plot_2d.launch.py\n"
	@printf "  landmark-vo-fisheye - build $(RADIO_PKG) + source + landmark_vo_plot_fisheye.launch.py\n"
	@printf "  radio-stack         - build $(RADIO_PKG) + source + radio_stack.launch.py PRESET=full|plots|vio|state|decoder\n"
	@printf "  rtl-adsb           - build $(RADIO_PKG) + source + rtl_adsb.launch.py\n"
	@printf "  adsb-state         - adsb_aircraft_state_vectors.launch.py ($(RADIO_PKG))\n"
	@printf "  adsb-glide         - adsb_state_vectors_plot_glide.launch.py ($(RADIO_PKG))\n"
	@printf "  sdr                - sdr.launch.py ($(RADIO_PKG))\n"
	@printf "\n"
	@printf "  system-launch      - sequenced MAVROS -> radio -> cameras -> spectrometer\n"
	@printf "  mavros             - mavros px4.launch (FCU+GCS from PIXHAWK_*/GCS_URL vars)\n"
	@printf "  mavros-bg          - same, in background via systemd --user\n"
	@printf "  gh-cam             - Grasshopper stereo cameras\n"
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
	@echo "RADIO_PKG        = $(RADIO_PKG)"
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

build-radio:
	cd $(ROS2_WS) && bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --packages-select $(RADIO_PKG) --paths $(RADIO_PKG_SRC) && source install/setup.bash'

landmark-vo:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(RADIO_PKG) --paths $(RADIO_PKG_SRC) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) landmark_vo_plot_2d.launch.py estimated_position_topic:=/adsb/rtl_adsb_decoder_node/estimated_position'

landmark-vo-fisheye:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(RADIO_PKG) --paths $(RADIO_PKG_SRC) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) landmark_vo_plot_fisheye.launch.py'

radio-stack:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(RADIO_PKG) --paths $(RADIO_PKG_SRC) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) radio_stack.launch.py preset:=$(if $(PRESET),$(PRESET),full)'

rtl-adsb:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && colcon build --packages-select $(RADIO_PKG) --paths $(RADIO_PKG_SRC) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) rtl_adsb.launch.py'

adsb-state:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) adsb_aircraft_state_vectors.launch.py'

adsb-glide:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) adsb_state_vectors_plot_glide.launch.py'

sdr:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && cd $(ROS2_WS) && source install/setup.bash && exec ros2 launch $(RADIO_PKG) sdr.launch.py'

# ---- MAVROS -----------------------------------------------------------------

system-launch:
	$(eval CMD := exec ros2 launch $(PKG) earth_rover_system.launch.py fcu_url:="$(PIXHAWK_DEVICE):$(PIXHAWK_BAUD)" gcs_url:="$(GCS_URL)")
	$(WITH_ROS)

mavros:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash; \
		exec ros2 launch mavros px4.launch fcu_url:="$(PIXHAWK_DEVICE):$(PIXHAWK_BAUD)" gcs_url:="$(GCS_URL)"'

mavros-bg:
	systemctl --user start er-mavros.service
	@echo "Started er-mavros.service.  Logs: journalctl --user -u er-mavros -f"

# ---- Cameras / sensors -------------------------------------------------------

gh-cam-left:
	$(eval CMD := exec ros2 launch spinnaker_camera_driver grasshopper_left.launch.py serial:=22312692 parameter_file:=grasshopper.yaml)
	$(WITH_ROS)

gh-cam-right:
	$(eval CMD := exec ros2 launch spinnaker_camera_driver grasshopper_right.launch.py serial:=22312674 parameter_file:=grasshopper.yaml)
	$(WITH_ROS)

meta-cam:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch metavision_driver driver_node.launch.py'

velo:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py'

rs-cam:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch realsense2_camera rs_launch.py'

spinnaker-left:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch spinnaker_camera_driver grasshopper_left.launch.py'

spinnaker-right:
	bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash && exec ros2 launch spinnaker_camera_driver grasshopper_right.launch.py'

# ---- rosbag recording --------------------------------------------------------
# Wraps deepgis_vehicles/launch/record_bag.launch.py.  Override per-class flags
# or any of the bag args via RECORD_ARGS, e.g.:
#   make record-bag RECORD_ARGS="record_stereo_raw:=true compression_mode:=file"
RECORD_ARGS ?=

record-bag:
	$(eval CMD := exec ros2 launch $(PKG) record_bag.launch.py $(RECORD_ARGS))
	$(WITH_ROS)

record-bag-mavros:
	$(eval CMD := exec ros2 launch $(PKG) record_bag.launch.py \
		record_stereo_compressed:=false \
		record_stereo_raw:=false \
		record_stereo_camera_info:=false \
		record_spectrometer:=false \
		record_laser:=false \
		record_adsb:=false $(RECORD_ARGS))
	$(WITH_ROS)

record-bag-stereo:
	$(eval CMD := exec ros2 launch $(PKG) record_bag.launch.py \
		record_mavros_state:=false \
		record_mavros_imu:=false \
		record_mavros_local_position:=false \
		record_mavros_global_position:=false \
		record_mavros_gps_status:=false \
		record_spectrometer:=false \
		record_laser:=false \
		record_adsb:=false \
		record_diagnostics:=false $(RECORD_ARGS))
	$(WITH_ROS)

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

# ---- Hotkeys ----------------------------------------------------------------
# Operator-facing keyboard shortcuts that wrap system-launch and
# record-bag-mavros in transient `er-hotkey-*` user units. See
# scripts/hotkeys/README.md for design + alternative bindings.

HOTKEYS_DIR := $(EARTH_ROVER_HOME)/scripts/hotkeys

hotkeys-install:
	bash $(HOTKEYS_DIR)/install-gnome.sh

hotkeys-uninstall:
	bash $(HOTKEYS_DIR)/uninstall-gnome.sh

hotkeys-status:
	bash $(HOTKEYS_DIR)/status.sh

hotkey-trike-start:
	bash $(HOTKEYS_DIR)/trike-start.sh

hotkey-trike-stop:
	bash $(HOTKEYS_DIR)/trike-stop.sh

hotkey-bag-start:
	bash $(HOTKEYS_DIR)/rosbag-start.sh

hotkey-bag-stop:
	bash $(HOTKEYS_DIR)/rosbag-stop.sh

hotkey-tail-trike:
	journalctl --user -fu er-hotkey-trike.service

hotkey-tail-bag:
	journalctl --user -fu er-hotkey-bag.service
