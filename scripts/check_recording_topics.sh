#!/bin/bash
# Check which recording topics are currently active
# Helps verify sensors are running before starting rosbag recording

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Topics to check (from record_Salient_topics.sh)
CAMERA_TOPICS=(
    # RealSense
    "/camera/fisheye1/image_raw"
    "/camera/fisheye2/image_raw"
    "/camera/imu"
    # Legacy cameras
    "/left/image_raw"
    "/right/image_raw"
    # Grasshopper3 Stereo
    "/stereo/left/image_raw"
    "/stereo/left/camera_info"
    "/stereo/right/image_raw"
    "/stereo/right/camera_info"
)

LIDAR_TOPICS=(
    "/velodyne_packets"
    "/velodyne_points"
    "/scan"
)

MAVROS_TOPICS=(
    "/mavros_trike/global_position/raw/fix"
    "/mavros_trike/imu/data"
    "/mavros_trike/local_position/odom"
    "/mavros_trike/state"
)

OTHER_TOPICS=(
    "/serial_data"
    "/spectrometer"
    "/tf"
    "/tf_static"
)

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}   ROS2 Topic Status Check for Rosbag Recording${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""

# Get list of active topics
echo -e "${YELLOW}Fetching active topics...${NC}"
ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null)

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Could not fetch ROS2 topics. Is ROS2 running?${NC}"
    exit 1
fi

# Function to check if topic exists
check_topic() {
    local topic=$1
    if echo "$ACTIVE_TOPICS" | grep -q "^${topic}$"; then
        return 0  # Topic exists
    else
        return 1  # Topic doesn't exist
    fi
}

# Function to get topic frequency
get_frequency() {
    local topic=$1
    timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A"
}

# Function to display topics status
display_topic_group() {
    local title=$1
    shift
    local topics=("$@")
    local active_count=0
    local total_count=${#topics[@]}
    
    echo ""
    echo -e "${CYAN}━━━ $title ━━━${NC}"
    
    for topic in "${topics[@]}"; do
        if check_topic "$topic"; then
            echo -e "  ${GREEN}✓${NC} $topic"
            active_count=$((active_count + 1))
        else
            echo -e "  ${RED}✗${NC} $topic"
        fi
    done
    
    echo -e "  ${YELLOW}Active: $active_count/$total_count${NC}"
}

# Display status for each group
display_topic_group "Camera Topics" "${CAMERA_TOPICS[@]}"
display_topic_group "LiDAR Topics" "${LIDAR_TOPICS[@]}"
display_topic_group "MAVROS Topics" "${MAVROS_TOPICS[@]}"
display_topic_group "Other Sensors" "${OTHER_TOPICS[@]}"

# Summary
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}   Summary${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"

TOTAL_CHECKED=$((${#CAMERA_TOPICS[@]} + ${#LIDAR_TOPICS[@]} + ${#MAVROS_TOPICS[@]} + ${#OTHER_TOPICS[@]}))
TOTAL_ACTIVE=0

for topic in "${CAMERA_TOPICS[@]}" "${LIDAR_TOPICS[@]}" "${MAVROS_TOPICS[@]}" "${OTHER_TOPICS[@]}"; do
    if check_topic "$topic"; then
        TOTAL_ACTIVE=$((TOTAL_ACTIVE + 1))
    fi
done

echo -e "Total topics checked: ${YELLOW}$TOTAL_CHECKED${NC}"
echo -e "Active topics:        ${GREEN}$TOTAL_ACTIVE${NC}"
echo -e "Inactive topics:      ${RED}$((TOTAL_CHECKED - TOTAL_ACTIVE))${NC}"

# Check critical sensors
echo ""
echo -e "${CYAN}Critical Sensor Status:${NC}"

GRASSHOPPER_LEFT=$(check_topic "/stereo/left/image_raw" && echo "✓" || echo "✗")
GRASSHOPPER_RIGHT=$(check_topic "/stereo/right/image_raw" && echo "✓" || echo "✗")
VELODYNE=$(check_topic "/velodyne_points" && echo "✓" || echo "✗")
TF=$(check_topic "/tf" && echo "✓" || echo "✗")

if [ "$GRASSHOPPER_LEFT" = "✓" ] && [ "$GRASSHOPPER_RIGHT" = "✓" ]; then
    echo -e "  ${GREEN}✓${NC} Grasshopper3 Stereo Camera: READY"
else
    echo -e "  ${RED}✗${NC} Grasshopper3 Stereo Camera: NOT READY"
fi

if [ "$VELODYNE" = "✓" ]; then
    echo -e "  ${GREEN}✓${NC} Velodyne LiDAR: READY"
else
    echo -e "  ${RED}✗${NC} Velodyne LiDAR: NOT READY"
fi

if [ "$TF" = "✓" ]; then
    echo -e "  ${GREEN}✓${NC} Transform Broadcaster: READY"
else
    echo -e "  ${RED}✗${NC} Transform Broadcaster: NOT READY"
fi

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"

# Check frame rates for active camera topics
echo ""
echo -e "${YELLOW}Do you want to check frame rates? (This takes ~3 seconds per topic)${NC}"
read -p "Check frame rates? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${CYAN}Checking frame rates...${NC}"
    
    if check_topic "/stereo/left/image_raw"; then
        echo -ne "  /stereo/left/image_raw: "
        HZ=$(get_frequency "/stereo/left/image_raw")
        if [ "$HZ" != "N/A" ]; then
            echo -e "${GREEN}${HZ} Hz${NC}"
        else
            echo -e "${RED}No data${NC}"
        fi
    fi
    
    if check_topic "/stereo/right/image_raw"; then
        echo -ne "  /stereo/right/image_raw: "
        HZ=$(get_frequency "/stereo/right/image_raw")
        if [ "$HZ" != "N/A" ]; then
            echo -e "${GREEN}${HZ} Hz${NC}"
        else
            echo -e "${RED}No data${NC}"
        fi
    fi
    
    if check_topic "/velodyne_points"; then
        echo -ne "  /velodyne_points: "
        HZ=$(get_frequency "/velodyne_points")
        if [ "$HZ" != "N/A" ]; then
            echo -e "${GREEN}${HZ} Hz${NC}"
        else
            echo -e "${RED}No data${NC}"
        fi
    fi
fi

echo ""
if [ $TOTAL_ACTIVE -ge 5 ]; then
    echo -e "${GREEN}✓ System looks good! Ready to record.${NC}"
    echo -e "Run: ${CYAN}~/record_Salient_topics.sh${NC}"
else
    echo -e "${YELLOW}⚠ Only $TOTAL_ACTIVE topics active. Start sensors before recording.${NC}"
fi

echo ""

