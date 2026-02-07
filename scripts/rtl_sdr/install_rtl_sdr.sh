#!/bin/bash
# ============================================================================
# Earth Rover - RTL-SDR V4 Dependencies Installation
# ============================================================================
#
# Installs all libraries and tools needed to run the RTL-SDR Blog V4
# software-defined radio nodes and ADS-B aircraft decoder.
#
# Components installed:
#   1. System packages (libusb, cmake, build tools)
#   2. rtl-sdr-blog drivers (V4 fork with bias-T, R828D support)
#   3. dump1090 ADS-B decoder (high-performance C decoder)
#   4. Python libraries (numpy, scipy, pyModeS, PyQt5, pyqtgraph)
#   5. udev rules for non-root USB access
#   6. Blacklist kernel DVB driver that conflicts with rtl-sdr
#
# Usage:
#   ./install_rtl_sdr.sh                  # Full install
#   ./install_rtl_sdr.sh --no-dump1090    # Skip dump1090
#   ./install_rtl_sdr.sh --no-gui         # Skip PyQt5/pyqtgraph
#   ./install_rtl_sdr.sh --deps-only      # System + Python deps only
#   ./install_rtl_sdr.sh --apt-only       # Use apt rtl-sdr (not V4 fork)
#
# Tested on: Ubuntu 22.04 / 24.04 with ROS2 Humble / Jazzy
# ============================================================================

set -e

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EARTH_ROVER_HOME="$(dirname "$(dirname "$SCRIPT_DIR")")"
BUILD_DIR="${EARTH_ROVER_HOME}/scripts/rtl_sdr/.build"

RTLSDR_BLOG_REPO="https://github.com/rtlsdrblog/rtl-sdr-blog.git"
DUMP1090_REPO="https://github.com/antirez/dump1090.git"

# Detect ROS2 distro
if [ -n "$ROS_DISTRO" ]; then
    ROS2_DISTRO="$ROS_DISTRO"
else
    for distro in jazzy iron humble rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            ROS2_DISTRO="$distro"
            break
        fi
    done
fi

# Parse flags
INSTALL_DUMP1090=true
INSTALL_GUI=true
DEPS_ONLY=false
APT_ONLY=false

for arg in "$@"; do
    case $arg in
        --no-dump1090)  INSTALL_DUMP1090=false ;;
        --no-gui)       INSTALL_GUI=false ;;
        --deps-only)    DEPS_ONLY=true ;;
        --apt-only)     APT_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-dump1090  Skip dump1090 ADS-B decoder build"
            echo "  --no-gui       Skip PyQt5/pyqtgraph (visualizer) dependencies"
            echo "  --deps-only    Install only system + Python deps (no source builds)"
            echo "  --apt-only     Use apt rtl-sdr package instead of V4 blog fork"
            echo "  --help         Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
ok()    { echo -e "${GREEN}[  OK]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $1"; }

# ============================================================================
# Banner
# ============================================================================

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     Earth Rover - RTL-SDR V4 + ADS-B Installer            ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║  RTL-SDR Blog V4: 24-1766 MHz, up to 3.2 MSPS             ║"
echo "║  ADS-B Decoder:   dump1090 at 1090 MHz                    ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
info "Earth Rover home:  ${EARTH_ROVER_HOME}"
info "ROS2 distro:       ${ROS2_DISTRO:-not detected}"
info "Install dump1090:  ${INSTALL_DUMP1090}"
info "Install GUI deps:  ${INSTALL_GUI}"
info "Deps only:         ${DEPS_ONLY}"
info "APT rtl-sdr only:  ${APT_ONLY}"
echo ""

# ============================================================================
# Step 1: System Packages
# ============================================================================

STEP=1
TOTAL=7
[ "$DEPS_ONLY" = true ] && TOTAL=5

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

if [ "$INSTALL_GUI" = true ]; then
    SYSTEM_PKGS+=(
        python3-pyqt5
        libxcb-xinerama0
        libxkbcommon-x11-0
    )
fi

# If apt-only, add rtl-sdr from distribution repos
if [ "$APT_ONLY" = true ]; then
    SYSTEM_PKGS+=(
        rtl-sdr
        librtlsdr-dev
    )
fi

info "Updating apt package index..."
sudo apt-get update -qq

info "Installing: ${SYSTEM_PKGS[*]}"
sudo apt-get install -y -qq "${SYSTEM_PKGS[@]}" 2>/dev/null
ok "System packages installed"

# ============================================================================
# Step 2: Blacklist kernel DVB driver (conflicts with rtl-sdr)
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Blacklisting conflicting kernel DVB driver"
echo "════════════════════════════════════════════════════════════"

BLACKLIST_FILE="/etc/modprobe.d/rtlsdr-blacklist.conf"

BLACKLIST_CONTENT='# Blacklist DVB-T drivers that conflict with rtl-sdr userspace driver
# The RTL2832U chipset is used by both DVB-T TV receivers and rtl-sdr.
# The kernel DVB driver must be blacklisted to allow rtl-sdr access.
blacklist dvb_usb_rtl28xxu
blacklist dvb_usb_rtl2832u
blacklist dvb_usb_v2
blacklist rtl2832
blacklist rtl2830
blacklist r820t'

if [ ! -f "$BLACKLIST_FILE" ]; then
    info "Creating DVB driver blacklist..."
    echo "$BLACKLIST_CONTENT" | sudo tee "$BLACKLIST_FILE" > /dev/null
    ok "Blacklist created at ${BLACKLIST_FILE}"
    warn "You may need to reboot for blacklist to take effect."
    warn "Or run: sudo rmmod dvb_usb_rtl28xxu 2>/dev/null"
else
    ok "DVB driver blacklist already exists"
fi

# Try to unload conflicting module now
sudo rmmod dvb_usb_rtl28xxu 2>/dev/null && \
    ok "Unloaded dvb_usb_rtl28xxu kernel module" || \
    info "dvb_usb_rtl28xxu not loaded (good)"

# ============================================================================
# Step 3: USB / udev rules for RTL-SDR
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Configuring USB access (udev rules)"
echo "════════════════════════════════════════════════════════════"

UDEV_RULE_FILE="/etc/udev/rules.d/99-rtlsdr.rules"

UDEV_RULES='# RTL-SDR Blog V4 and compatible devices -- non-root USB access
# RTL2832U based devices
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="2832", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="2838", MODE="0666", GROUP="plugdev"
# RTL-SDR Blog branded
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="2832", MODE="0666", GROUP="plugdev"
# Generic RTL2832U
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="2834", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="2837", MODE="0666", GROUP="plugdev"
# Terratec Cinergy T Stick
SUBSYSTEM=="usb", ATTR{idVendor}=="0ccd", ATTR{idProduct}=="00a9", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="0ccd", ATTR{idProduct}=="00b3", MODE="0666", GROUP="plugdev"
# Nooelec NESDR
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="2838", MODE="0666", GROUP="plugdev"'

if [ ! -f "$UDEV_RULE_FILE" ]; then
    info "Creating udev rules for RTL-SDR USB access..."
    echo "$UDEV_RULES" | sudo tee "$UDEV_RULE_FILE" > /dev/null
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
# Step 4: Python Libraries
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

python3 -c "import numpy; print(f'  numpy        {numpy.__version__}')" 2>/dev/null \
    && ok "numpy" || { warn "numpy import failed"; }

python3 -c "import scipy; print(f'  scipy        {scipy.__version__}')" 2>/dev/null \
    && ok "scipy" || { warn "scipy import failed (signal filtering degraded)"; }

python3 -c "import pyModeS; print(f'  pyModeS      {pyModeS.__version__}')" 2>/dev/null \
    && ok "pyModeS" || { warn "pyModeS import failed -- IQ-mode ADS-B decoding unavailable"; }

if [ "$INSTALL_GUI" = true ]; then
    python3 -c "import pyqtgraph; print(f'  pyqtgraph    {pyqtgraph.__version__}')" 2>/dev/null \
        && ok "pyqtgraph" || { warn "pyqtgraph import failed -- visualizer unavailable"; }
    python3 -c "import PyQt5; print('  PyQt5        OK')" 2>/dev/null \
        && ok "PyQt5" || { warn "PyQt5 import failed -- visualizer unavailable"; }
fi

# ============================================================================
# Step 5: ROS2 Dependencies
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
    warn "ROS2 distro not detected. Source your ROS2 setup.bash and re-run."
fi

# Exit early if deps-only
if [ "$DEPS_ONLY" = true ]; then
    echo ""
    ok "Deps-only mode complete. Skipping source builds."
    echo ""
    echo "  To build rtl-sdr-blog from source, re-run without --deps-only."
    exit 0
fi

# ============================================================================
# Step 6: Build rtl-sdr-blog (V4 fork) from source
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Building rtl-sdr-blog (V4 fork) from source"
echo "════════════════════════════════════════════════════════════"

if [ "$APT_ONLY" = true ]; then
    info "Using apt rtl-sdr package (--apt-only flag). Skipping source build."
    if command -v rtl_sdr &>/dev/null; then
        ok "rtl_sdr is available"
    else
        warn "rtl_sdr not found on PATH"
    fi
else
    mkdir -p "$BUILD_DIR"

    if command -v rtl_sdr &>/dev/null; then
        # Check if it's the blog fork by looking for rtl_biast
        if command -v rtl_biast &>/dev/null; then
            ok "rtl-sdr-blog (V4 fork) already installed"
        else
            info "rtl_sdr found but rtl_biast missing -- may be stock rtl-sdr"
            info "Building rtl-sdr-blog fork for full V4 support..."
        fi
    fi

    RTLSDR_SRC="${BUILD_DIR}/rtl-sdr-blog"

    if [ -d "$RTLSDR_SRC" ]; then
        info "Updating existing rtl-sdr-blog source..."
        cd "$RTLSDR_SRC"
        git pull --ff-only 2>/dev/null || true
    else
        info "Cloning rtl-sdr-blog (V4 fork)..."
        git clone "$RTLSDR_BLOG_REPO" "$RTLSDR_SRC" 2>/dev/null || {
            warn "Could not clone from ${RTLSDR_BLOG_REPO}"
            warn "Trying official rtl-sdr as fallback..."
            git clone https://github.com/osmocom/rtl-sdr.git "$RTLSDR_SRC" 2>/dev/null || {
                fail "Could not clone rtl-sdr from any source"
                RTLSDR_SRC=""
            }
        }
    fi

    if [ -n "$RTLSDR_SRC" ] && [ -d "$RTLSDR_SRC" ]; then
        info "Building rtl-sdr..."
        cd "$RTLSDR_SRC"
        mkdir -p build && cd build
        cmake .. \
            -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DINSTALL_UDEV_RULES=ON \
            -DDETACH_KERNEL_DRIVER=ON
        make -j"$(nproc)"
        sudo make install
        sudo ldconfig

        # Copy udev rules from source if present
        if [ -f "$RTLSDR_SRC/rtl-sdr.rules" ]; then
            sudo cp "$RTLSDR_SRC/rtl-sdr.rules" /etc/udev/rules.d/20-rtlsdr.rules
            sudo udevadm control --reload-rules
            sudo udevadm trigger
        fi

        ok "rtl-sdr-blog installed to /usr/local"

        # Verify installation
        echo ""
        info "Verifying rtl-sdr tools..."
        command -v rtl_sdr    &>/dev/null && ok "rtl_sdr"    || warn "rtl_sdr not on PATH"
        command -v rtl_test   &>/dev/null && ok "rtl_test"   || warn "rtl_test not on PATH"
        command -v rtl_biast  &>/dev/null && ok "rtl_biast"  || warn "rtl_biast not found (V4 bias-T unavailable)"
        command -v rtl_eeprom &>/dev/null && ok "rtl_eeprom" || info "rtl_eeprom not found (optional)"
        command -v rtl_power  &>/dev/null && ok "rtl_power"  || info "rtl_power not found (optional)"
    fi
fi

# ============================================================================
# Step 7: Build dump1090 ADS-B decoder
# ============================================================================

STEP=$((STEP + 1))
echo ""
echo "════════════════════════════════════════════════════════════"
echo " [${STEP}/${TOTAL}] Building dump1090 ADS-B decoder"
echo "════════════════════════════════════════════════════════════"

if [ "$INSTALL_DUMP1090" = true ]; then
    mkdir -p "$BUILD_DIR"

    if command -v dump1090 &>/dev/null; then
        ok "dump1090 already installed"
    else
        # Check for alternatives
        for alt in dump1090-mutability dump1090-fa readsb; do
            if command -v "$alt" &>/dev/null; then
                ok "$alt found (compatible dump1090 variant)"
                INSTALL_DUMP1090=false
                break
            fi
        done
    fi

    if [ "$INSTALL_DUMP1090" = true ] && ! command -v dump1090 &>/dev/null; then
        DUMP1090_SRC="${BUILD_DIR}/dump1090"

        # Need librtlsdr for dump1090
        if [ ! -f /usr/local/lib/librtlsdr.so ] && [ ! -f /usr/lib/librtlsdr.so ] && \
           [ ! -f /usr/lib/x86_64-linux-gnu/librtlsdr.so ]; then
            warn "librtlsdr not found -- dump1090 build may fail"
            warn "Make sure rtl-sdr is installed first"
        fi

        if [ -d "$DUMP1090_SRC" ]; then
            info "Updating existing dump1090 source..."
            cd "$DUMP1090_SRC"
            git pull --ff-only 2>/dev/null || true
        else
            info "Cloning dump1090..."
            git clone "$DUMP1090_REPO" "$DUMP1090_SRC" 2>/dev/null || {
                warn "Could not clone dump1090 from ${DUMP1090_REPO}"
                warn "Trying flightaware fork..."
                git clone https://github.com/flightaware/dump1090.git "$DUMP1090_SRC" 2>/dev/null || {
                    fail "Could not clone dump1090 from any source"
                    DUMP1090_SRC=""
                }
            }
        fi

        if [ -n "$DUMP1090_SRC" ] && [ -d "$DUMP1090_SRC" ]; then
            info "Building dump1090..."
            cd "$DUMP1090_SRC"

            # dump1090 uses a plain Makefile
            if [ -f Makefile ]; then
                make clean 2>/dev/null || true
                make -j"$(nproc)" 2>/dev/null || {
                    warn "dump1090 build failed -- trying with PKG_CONFIG_PATH"
                    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH \
                        CFLAGS="-I/usr/local/include" \
                        LDFLAGS="-L/usr/local/lib" \
                        make -j"$(nproc)" || {
                            fail "dump1090 build failed"
                            fail "You can install dump1090-mutability via apt instead:"
                            fail "  sudo apt install dump1090-mutability"
                        }
                }

                if [ -f dump1090 ]; then
                    sudo cp dump1090 /usr/local/bin/dump1090
                    sudo chmod +x /usr/local/bin/dump1090
                    ok "dump1090 installed to /usr/local/bin/dump1090"
                fi
            else
                warn "No Makefile found in dump1090 source -- skipping build"
            fi
        fi
    fi
else
    info "Skipping dump1090 (--no-dump1090 flag set)"
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
echo "  2. Plug in RTL-SDR V4 via USB and verify:"
echo "     rtl_test -t"
echo ""
echo "  3. Launch SDR spectrum monitoring:"
echo "     ros2 launch deepgis_vehicles rtl_sdr.launch.py"
echo ""
echo "  4. Launch ADS-B aircraft decoder (dump1090 mode):"
echo "     ros2 launch deepgis_vehicles rtl_adsb.launch.py"
echo ""
echo "  5. Launch ADS-B in IQ mode (pyModeS, no dump1090):"
echo "     ros2 launch deepgis_vehicles rtl_adsb.launch.py decoder_mode:=iq"
echo ""

# Warnings
if ! command -v rtl_sdr &>/dev/null; then
    warn "rtl_sdr not found on PATH -- SDR streaming will not work"
fi
if ! command -v dump1090 &>/dev/null; then
    FOUND_ALT=false
    for alt in dump1090-mutability dump1090-fa readsb; do
        if command -v "$alt" &>/dev/null; then
            info "Using $alt instead of dump1090"
            FOUND_ALT=true
            break
        fi
    done
    if [ "$FOUND_ALT" = false ]; then
        warn "dump1090 not found -- use decoder_mode:=iq for ADS-B"
    fi
fi

echo ""
echo "  Troubleshooting:"
echo "    - 'device busy': sudo rmmod dvb_usb_rtl28xxu"
echo "    - 'permission denied': re-login for plugdev group, or use sudo"
echo "    - Reboot recommended after first install for udev + blacklist"
echo ""
echo "  For help: $0 --help"
echo ""
