#!/bin/bash
# ============================================================================
# Earth Rover - HydraSDR + ADS-B Dependencies Installation
# ============================================================================
#
# Installs all libraries and tools needed to run the HydraSDR RFOne
# software-defined radio nodes and ADS-B aircraft decoder.
#
# Components installed:
#   1. System packages (libusb, cmake, build tools, udev rules)
#   2. hydrasdr-host CLI tools (hydrasdr_info, hydrasdr_rx, hydrasdr_async_rx)
#   3. SoapySDR framework + SoapyHydraSDR plugin (optional)
#   4. Python libraries (numpy, scipy, pyModeS, PyQt5, pyqtgraph)
#   5. ROS2 package build
#
# Usage:
#   ./install_hydra_sdr.sh              # Full install
#   ./install_hydra_sdr.sh --no-soapy   # Skip SoapySDR
#   ./install_hydra_sdr.sh --no-gui     # Skip PyQt5/pyqtgraph visualizer deps
#   ./install_hydra_sdr.sh --deps-only  # Python + system deps only (no source builds)
#
# Tested on: Ubuntu 22.04 / 24.04 with ROS2 Humble / Jazzy
# ============================================================================

set -e

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EARTH_ROVER_HOME="$(dirname "$(dirname "$SCRIPT_DIR")")"
BUILD_DIR="${EARTH_ROVER_HOME}/scripts/hydra_sdr/.build"

HYDRASDR_HOST_REPO="https://github.com/hydrasdr/hydrasdr-host.git"
SOAPY_SDR_REPO="https://github.com/pothosware/SoapySDR.git"
SOAPY_HYDRA_REPO="https://github.com/hydrasdr/SoapyHydraSDR.git"

# Detect ROS2 distro
if [ -n "$ROS_DISTRO" ]; then
    ROS2_DISTRO="$ROS_DISTRO"
else
    # Try to detect from installed ROS2
    for distro in jazzy iron humble rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            ROS2_DISTRO="$distro"
            break
        fi
    done
fi

# Parse flags
INSTALL_SOAPY=true
INSTALL_GUI=true
DEPS_ONLY=false

for arg in "$@"; do
    case $arg in
        --no-soapy)  INSTALL_SOAPY=false ;;
        --no-gui)    INSTALL_GUI=false ;;
        --deps-only) DEPS_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-soapy   Skip SoapySDR + SoapyHydraSDR installation"
            echo "  --no-gui     Skip PyQt5/pyqtgraph (visualizer) dependencies"
            echo "  --deps-only  Install only system + Python deps (no source builds)"
            echo "  --help       Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
ok()    { echo -e "${GREEN}[  OK]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $1"; }

# ============================================================================
# Banner
# ============================================================================

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     Earth Rover - HydraSDR + ADS-B Installer              ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║  HydraSDR RFOne: 24-1800 MHz, up to 10 MSPS               ║"
echo "║  ADS-B Decoder:  1090 MHz aircraft tracking                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
info "Earth Rover home:  ${EARTH_ROVER_HOME}"
info "ROS2 distro:       ${ROS2_DISTRO:-not detected}"
info "Install SoapySDR:  ${INSTALL_SOAPY}"
info "Install GUI deps:  ${INSTALL_GUI}"
info "Deps only:         ${DEPS_ONLY}"
echo ""

# ============================================================================
# Step 1: System Packages
# ============================================================================

STEP=1
TOTAL=6
[ "$DEPS_ONLY" = true ] && TOTAL=4

echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Installing system packages"
echo "════════════════════════════════════════════════════════════"

SYSTEM_PKGS=(
    build-essential
    cmake
    pkg-config
    git
    libusb-1.0-0-dev
    libusb-1.0-0
    udev
    python3-pip
    python3-dev
    python3-numpy
)

# Add GUI packages if requested
if [ "$INSTALL_GUI" = true ]; then
    SYSTEM_PKGS+=(
        python3-pyqt5
        libxcb-xinerama0
        libxkbcommon-x11-0
    )
fi

# Add SoapySDR system packages if requested
if [ "$INSTALL_SOAPY" = true ]; then
    SYSTEM_PKGS+=(
        libsoapysdr-dev
        soapysdr-tools
    )
fi

info "Updating apt package index..."
sudo apt-get update -qq

info "Installing: ${SYSTEM_PKGS[*]}"
sudo apt-get install -y -qq "${SYSTEM_PKGS[@]}" 2>/dev/null
ok "System packages installed"

# ============================================================================
# Step 2: USB / udev rules for HydraSDR
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Configuring USB access (udev rules)"
echo "════════════════════════════════════════════════════════════"

UDEV_RULE_FILE="/etc/udev/rules.d/99-hydrasdr.rules"

# HydraSDR RFOne known USB vendor:product IDs
# Airspy/HydraSDR uses VID 0x1d50 (OpenMoko) PID 0x604b
UDEV_RULE='# HydraSDR RFOne - allow non-root USB access
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="604b", MODE="0666", GROUP="plugdev"
# Airspy HF+ (alternate ID used by some HydraSDR variants)
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="60a1", MODE="0666", GROUP="plugdev"'

if [ ! -f "$UDEV_RULE_FILE" ]; then
    info "Creating udev rules for HydraSDR USB access..."
    echo "$UDEV_RULE" | sudo tee "$UDEV_RULE_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ok "udev rules installed at ${UDEV_RULE_FILE}"
else
    ok "udev rules already exist at ${UDEV_RULE_FILE}"
fi

# Ensure user is in plugdev group
if ! groups "$USER" | grep -q plugdev; then
    info "Adding $USER to plugdev group..."
    sudo usermod -aG plugdev "$USER"
    warn "You may need to log out and back in for group changes to take effect"
else
    ok "User $USER is already in plugdev group"
fi

# ============================================================================
# Step 3: Python Libraries
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Installing Python libraries"
echo "════════════════════════════════════════════════════════════"

# --------------------------------------------------------------------------
# Strategy: Use apt for packages available as system debs, then pip (with
# --break-system-packages if needed) for the remaining PyPI-only packages.
# This avoids the PEP 668 "externally-managed-environment" error on
# Ubuntu 24.04+ / Python 3.12+.
# --------------------------------------------------------------------------

APT_PY_PKGS=(
    python3-numpy
    python3-scipy
)

if [ "$INSTALL_GUI" = true ]; then
    APT_PY_PKGS+=(
        python3-pyqt5
        python3-pyqtgraph
    )
fi

info "Installing system Python packages via apt: ${APT_PY_PKGS[*]}"
sudo apt-get install -y -qq "${APT_PY_PKGS[@]}" 2>/dev/null
ok "System Python packages installed via apt"

# PyPI-only packages (not available as Ubuntu debs)
PIP_PKGS=(
    pyModeS
)

info "Installing PyPI-only packages: ${PIP_PKGS[*]}"

# Detect if we need --break-system-packages (PEP 668, Python 3.12+)
PIP_EXTRA_FLAGS=""
if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3,12) else 1)" 2>/dev/null; then
    # Check if we're in a venv already
    if [ -z "$VIRTUAL_ENV" ]; then
        PIP_EXTRA_FLAGS="--break-system-packages"
        info "Python >= 3.12 detected, using --break-system-packages for PyPI packages"
    fi
fi

pip3 install --user --upgrade $PIP_EXTRA_FLAGS "${PIP_PKGS[@]}"
ok "Python libraries installed"

# Verify critical imports
echo ""
info "Verifying Python imports..."

IMPORT_OK=true

python3 -c "import numpy; print(f'  numpy        {numpy.__version__}')" 2>/dev/null \
    && ok "numpy" || { warn "numpy import failed"; IMPORT_OK=false; }

python3 -c "import scipy; print(f'  scipy        {scipy.__version__}')" 2>/dev/null \
    && ok "scipy" || { warn "scipy import failed (filtering will be degraded)"; }

python3 -c "import pyModeS; print(f'  pyModeS      {pyModeS.__version__}')" 2>/dev/null \
    && ok "pyModeS" || { warn "pyModeS import failed - ADS-B decoding disabled"; IMPORT_OK=false; }

if [ "$INSTALL_GUI" = true ]; then
    python3 -c "import pyqtgraph; print(f'  pyqtgraph    {pyqtgraph.__version__}')" 2>/dev/null \
        && ok "pyqtgraph" || { warn "pyqtgraph import failed - visualizer unavailable"; }

    python3 -c "import PyQt5; print('  PyQt5        OK')" 2>/dev/null \
        && ok "PyQt5" || { warn "PyQt5 import failed - visualizer unavailable"; }
fi

# ============================================================================
# Step 4: ROS2 Dependencies
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Installing ROS2 dependencies"
echo "════════════════════════════════════════════════════════════"

if [ -n "$ROS2_DISTRO" ]; then
    ROS2_PKGS=(
        "ros-${ROS2_DISTRO}-std-msgs"
        "ros-${ROS2_DISTRO}-std-srvs"
        "ros-${ROS2_DISTRO}-sensor-msgs"
        "ros-${ROS2_DISTRO}-geometry-msgs"
        "ros-${ROS2_DISTRO}-nav-msgs"
    )

    info "Installing ROS2 message packages for ${ROS2_DISTRO}..."
    sudo apt-get install -y -qq "${ROS2_PKGS[@]}" 2>/dev/null
    ok "ROS2 dependencies installed"
else
    warn "ROS2 distro not detected. Source your ROS2 setup.bash and re-run, or install manually:"
    warn "  sudo apt install ros-<distro>-std-msgs ros-<distro>-std-srvs ros-<distro>-sensor-msgs"
fi

# Exit early if deps-only mode
if [ "$DEPS_ONLY" = true ]; then
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo " Deps-only mode: skipping source builds"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    ok "All dependency packages installed."
    echo ""
    echo "To build hydrasdr-host from source, re-run without --deps-only."
    exit 0
fi

# ============================================================================
# Step 5: Build hydrasdr-host from source
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Building hydrasdr-host tools from source"
echo "════════════════════════════════════════════════════════════"

mkdir -p "$BUILD_DIR"

if command -v hydrasdr_info &>/dev/null; then
    ok "hydrasdr-host tools already installed (hydrasdr_info found on PATH)"
    hydrasdr_info --version 2>/dev/null || true
else
    HYDRASDR_SRC="${BUILD_DIR}/hydrasdr-host"

    if [ -d "$HYDRASDR_SRC" ]; then
        info "Updating existing hydrasdr-host source..."
        cd "$HYDRASDR_SRC"
        git pull --ff-only 2>/dev/null || true
    else
        info "Cloning hydrasdr-host..."
        git clone "$HYDRASDR_HOST_REPO" "$HYDRASDR_SRC" 2>/dev/null || {
            warn "Could not clone hydrasdr-host from ${HYDRASDR_HOST_REPO}"
            warn "If the repo is private or renamed, clone it manually into:"
            warn "  ${HYDRASDR_SRC}"
            warn "Then re-run this script."
            warn "Skipping hydrasdr-host build."
            HYDRASDR_SRC=""
        }
    fi

    if [ -n "$HYDRASDR_SRC" ] && [ -d "$HYDRASDR_SRC" ]; then
        info "Building hydrasdr-host..."
        cd "$HYDRASDR_SRC"
        mkdir -p build && cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
        make -j"$(nproc)"
        sudo make install
        sudo ldconfig
        ok "hydrasdr-host installed to /usr/local"

        # Verify
        if command -v hydrasdr_info &>/dev/null; then
            ok "hydrasdr_info is available on PATH"
        else
            warn "hydrasdr_info not found on PATH after install"
            warn "You may need to add /usr/local/bin to PATH"
        fi
    fi
fi

# ============================================================================
# Step 6: Build SoapySDR + SoapyHydraSDR plugin (optional)
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] SoapySDR + SoapyHydraSDR plugin"
echo "════════════════════════════════════════════════════════════"

if [ "$INSTALL_SOAPY" = true ]; then
    # Check if SoapySDR is already usable
    if command -v SoapySDRUtil &>/dev/null; then
        ok "SoapySDR already installed"
        SoapySDRUtil --info 2>/dev/null | head -5 || true
    else
        info "SoapySDR not found - it was installed via apt above."
        info "If SoapySDRUtil is still not on PATH, you may need to build from source."
    fi

    # Build SoapyHydraSDR plugin
    SOAPY_HYDRA_SRC="${BUILD_DIR}/SoapyHydraSDR"

    if [ -d "$SOAPY_HYDRA_SRC" ]; then
        info "Updating existing SoapyHydraSDR source..."
        cd "$SOAPY_HYDRA_SRC"
        git pull --ff-only 2>/dev/null || true
    else
        info "Cloning SoapyHydraSDR plugin..."
        git clone "$SOAPY_HYDRA_REPO" "$SOAPY_HYDRA_SRC" 2>/dev/null || {
            warn "Could not clone SoapyHydraSDR from ${SOAPY_HYDRA_REPO}"
            warn "Skipping SoapyHydraSDR plugin build."
            SOAPY_HYDRA_SRC=""
        }
    fi

    if [ -n "$SOAPY_HYDRA_SRC" ] && [ -d "$SOAPY_HYDRA_SRC" ]; then
        info "Building SoapyHydraSDR plugin..."
        cd "$SOAPY_HYDRA_SRC"
        mkdir -p build && cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
        make -j"$(nproc)"
        sudo make install
        sudo ldconfig
        ok "SoapyHydraSDR plugin installed"

        # Verify SoapySDR can see the plugin
        if command -v SoapySDRUtil &>/dev/null; then
            info "Checking SoapySDR modules..."
            SoapySDRUtil --check=hydrasdr 2>/dev/null || warn "SoapySDR cannot find hydrasdr module"
        fi
    fi
else
    info "Skipping SoapySDR (--no-soapy flag set)"
fi

# ============================================================================
# Summary
# ============================================================================

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              Installation Complete                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
ok "All requested components have been installed."
echo ""
echo "  Next steps:"
echo ""
echo "  1. Build the ROS2 package:"
echo "     cd ${EARTH_ROVER_HOME}"
echo "     colcon build --packages-select deepgis_vehicles"
echo "     source install/setup.bash"
echo ""
echo "  2. Plug in HydraSDR RFOne via USB and verify:"
echo "     hydrasdr_info"
echo ""
echo "  3. Launch SDR nodes (general spectrum monitoring):"
echo "     ros2 launch deepgis_vehicles sdr.launch.py"
echo ""
echo "  4. Launch ADS-B aircraft decoder:"
echo "     ros2 launch deepgis_vehicles adsb.launch.py"
echo ""
echo "  5. Launch visualizer standalone:"
echo "     ros2 run deepgis_vehicles sdr_visualizer.py"
echo ""

if ! command -v hydrasdr_info &>/dev/null; then
    warn "hydrasdr_info is not on PATH."
    warn "The hydrasdr-host tools may have failed to build."
    warn "The SDR nodes will start but cannot stream without these tools."
    echo ""
fi

echo "  For help: $0 --help"
echo ""
