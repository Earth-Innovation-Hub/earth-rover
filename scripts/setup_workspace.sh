#!/usr/bin/env bash
# Earth Rover -- one-shot workspace setup.
#
# Bootstraps a ~/ros2_ws/ that builds this repo end-to-end, starting from a
# fresh checkout of `Earth-Innovation-Hub/earth-rover`.  Idempotent: re-running
# is a no-op once everything is in place.
#
# Steps:
#   1. clone+update the git submodules under external/  (the darknight-007
#      forks of deepgis_vision and metavision_driver)
#   2. symlink ~/ros2_ws/src/earth-rover -> this repo, so colcon discovers all
#      first-party packages (deepgis_vehicles, radio_vio, laser_ranger, rtlsdr_ros2, spectrometery_ros2,
#      and the external/ submodules)
#   3. vcs import the upstream pins from earth-rover.repos into ~/ros2_ws/src/
#   4. (optional) git apply external/patches/*.patch on top of the imported
#      sources, so the rover behaves identically to santacruz today
#   5. colcon build --symlink-install
#
# Usage:
#   scripts/setup_workspace.sh                    # everything
#   scripts/setup_workspace.sh --no-build         # skip colcon build
#   scripts/setup_workspace.sh --no-patches       # skip applying local patches
#   scripts/setup_workspace.sh --ws ~/other_ws    # use a different workspace root

set -euo pipefail

# ---- args ---------------------------------------------------------------
WS="${HOME}/ros2_ws"
DO_BUILD=1
DO_PATCHES=1

while [ $# -gt 0 ]; do
    case "$1" in
        --no-build)   DO_BUILD=0   ;;
        --no-patches) DO_PATCHES=0 ;;
        --ws)         shift; WS="$1" ;;
        --help|-h)
            sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

say() { printf '\n[setup-workspace] %s\n' "$*"; }

# ---- 1. submodules -------------------------------------------------------
say "syncing git submodules under external/ ..."
git submodule update --init --recursive --jobs 4

# ---- 2. symlink workspace -----------------------------------------------
#
# colcon's package crawler stops at the first package.xml it finds in any
# directory tree.  Because earth-rover/package.xml (deepgis_vehicles) lives
# at the repo root, colcon will NOT descend into earth-rover/packages/.  We
# work around that by exposing each first-party sub-package as a sibling
# symlink directly under ~/ros2_ws/src/, so colcon discovers them as
# top-level entries.  Same trick is used for the two darknight-007 forks
# that we maintain as git submodules under earth-rover/external/.
#
mkdir -p "$WS/src"

link_pkg() {
    # link_pkg <name> <target_dir>
    local name="$1" target="$2" link="$WS/src/$1"
    if [ -L "$link" ] && [ "$(readlink -f "$link")" = "$(readlink -f "$target")" ]; then
        say "symlink $link -> $target  (ok)"
    else
        say "creating symlink $link -> $target"
        rm -rf "$link"
        ln -s "$target" "$link"
    fi
}

link_pkg earth-rover         "$REPO_DIR"
link_pkg radio_vio           "$REPO_DIR/packages/radio_vio"
link_pkg laser_ranger        "$REPO_DIR/packages/laser_ranger"
link_pkg rtlsdr_ros2         "$REPO_DIR/packages/rtlsdr_ros2"
link_pkg spectrometery_ros2  "$REPO_DIR/packages/spectrometery_ros2"
link_pkg deepgis_vision      "$REPO_DIR/external/deepgis_vision"
link_pkg metavision_driver   "$REPO_DIR/external/metavision_driver"

# ---- 3. vcs import upstream pins ----------------------------------------
if ! command -v vcs >/dev/null 2>&1; then
    cat >&2 <<EOF
[setup-workspace] vcstool not installed.  Try:
   sudo apt install python3-vcstool
or:
   pip install --user vcstool
EOF
    exit 1
fi

say "vcs import of pinned upstream packages -> $WS/src/ ..."
( cd "$WS/src" && vcs import --skip-existing < "$REPO_DIR/earth-rover.repos" )

# ---- 4. apply local patches ---------------------------------------------
if [ "$DO_PATCHES" -eq 1 ]; then
    say "applying earth-rover/external/patches/*.patch on top of upstream pins ..."

    apply_patch() {
        # apply_patch <target_dir> <patch_file>
        # `.git` is a directory in regular checkouts and a FILE in submodules,
        # so we test with -e (exists) rather than -d (is-directory).
        local target="$1" patch="$2"
        if [ ! -f "$patch" ]; then return 0; fi
        if [ ! -e "$target/.git" ]; then
            echo "  $(basename "$target"): not a git checkout, skipping"
            return 0
        fi
        if git -C "$target" apply --check "$patch" >/dev/null 2>&1; then
            git -C "$target" apply "$patch"
            echo "  $(basename "$target"): patch applied"
        else
            echo "  $(basename "$target"): patch already applied or conflicts -- skipping"
        fi
    }

    apply_patch "$REPO_DIR/external/metavision_driver" \
                "$REPO_DIR/external/patches/metavision_driver.local.patch"

    apply_patch "$WS/src/orbslam3_ros2" \
                "$REPO_DIR/external/patches/orbslam3_ros2.local.patch"

    if [ -f "$REPO_DIR/external/patches/orbslam3_gh_monocular.yaml" ] \
       && [ -d "$WS/src/orbslam3_ros2" ]; then
        mkdir -p "$WS/src/orbslam3_ros2/config/monocular"
        cp -n "$REPO_DIR/external/patches/orbslam3_gh_monocular.yaml" \
              "$WS/src/orbslam3_ros2/config/monocular/gh.yaml" \
            && echo "  orbslam3_ros2: monocular config in place" \
            || echo "  orbslam3_ros2: monocular config already present"
    fi
fi

# ---- 5. colcon build ----------------------------------------------------
if [ "$DO_BUILD" -eq 1 ]; then
    if ! command -v colcon >/dev/null 2>&1; then
        echo "[setup-workspace] colcon not on PATH; skipping build" >&2
        exit 0
    fi

    # Source ROS underlay if available and not already sourced.
    if [ -z "${ROS_DISTRO:-}" ]; then
        for cand in /opt/ros/humble/setup.bash /opt/ros/iron/setup.bash /opt/ros/jazzy/setup.bash; do
            if [ -f "$cand" ]; then
                # shellcheck disable=SC1090
                . "$cand"
                break
            fi
        done
    fi

    say "colcon build --symlink-install in $WS ..."
    ( cd "$WS" && colcon build --symlink-install --merge-install )
    echo
    echo "[setup-workspace] DONE.  Source the workspace with:"
    echo "    source $WS/install/setup.bash"
fi
