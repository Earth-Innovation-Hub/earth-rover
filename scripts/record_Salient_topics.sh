#!/bin/bash
# Improved ROS2 Bag Recording Script
# Records salient topics with timestamped output and various options

set -e  # Exit on error

# Default values
OUTPUT_DIR="${HOME}/rosbags"
COMPRESSION="none"
MAX_DURATION=""
MAX_SIZE=""
RECORD_ALL=false
INCLUDE_HIDDEN=false
USE_SIM_TIME=false
LOG_LEVEL="info"

# Define default topics to record
# Updated based on currently published topics (as of $(date +"%Y-%m-%d"))
DEFAULT_TOPICS=(
    # Grasshopper3 Stereo Camera topics - Main sensor data
    "/stereo/left/camera_info"
    "/stereo/left/image_raw"
    "/stereo/left/meta"
    "/stereo/left/control"
    "/stereo/right/camera_info"
    "/stereo/right/image_raw"
    "/stereo/right/meta"
    "/stereo/right/control"
    
    # Velodyne LiDAR topics
    "/velodyne_packets"
    "/velodyne_points"
    "/scan"
    
    # System diagnostics
    "/diagnostics"
    
    # Transforms (essential for sensor fusion)
    "/tf"
    "/tf_static"
)

# Function to display usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS] [TOPICS...]

Record ROS2 topics to a bag file with timestamped output.

OPTIONS:
    -o, --output DIR          Output directory (default: ~/rosbags)
    -n, --name NAME           Custom bag name (default: auto-generated timestamp)
    -c, --compression MODE    Compression mode: none, file, or message (default: none)
    -f, --format FORMAT       Compression format: zstd (default: none)
    -d, --duration SECONDS    Maximum duration in seconds before splitting
    -s, --size BYTES          Maximum size in bytes before splitting (e.g., 1GB, 500MB)
    -a, --all                 Record all topics (ignores topic list)
    -h, --hidden              Include hidden topics
    --sim-time                Use simulation time
    --log-level LEVEL         Log level: debug, info, warn, error, fatal (default: info)
    --help                    Show this help message

EXAMPLES:
    $0                                    # Record default topics with timestamp
    $0 -o ~/my_bags -n test_run          # Custom output dir and name
    $0 -c file -f zstd                  # Record with file compression
    $0 -d 3600                           # Record for 1 hour max
    $0 -a                                # Record all topics
    $0 /velodyne_points /left/image_raw  # Record only specified topics

EOF
}

# Parse command-line arguments
CUSTOM_TOPICS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -n|--name)
            BAG_NAME="$2"
            shift 2
            ;;
        -c|--compression)
            COMPRESSION="$2"
            shift 2
            ;;
        -f|--format)
            COMPRESSION_FORMAT="$2"
            shift 2
            ;;
        -d|--duration)
            MAX_DURATION="$2"
            shift 2
            ;;
        -s|--size)
            MAX_SIZE="$2"
            shift 2
            ;;
        -a|--all)
            RECORD_ALL=true
            shift
            ;;
        -h|--hidden)
            INCLUDE_HIDDEN=true
            shift
            ;;
        --sim-time)
            USE_SIM_TIME=true
            shift
            ;;
        --log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            CUSTOM_TOPICS+=("$1")
            shift
            ;;
    esac
done

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Generate bag name with timestamp if not provided
if [ -z "$BAG_NAME" ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BAG_NAME="rosbag_${TIMESTAMP}"
fi

OUTPUT_PATH="${OUTPUT_DIR}/${BAG_NAME}"

# Build ros2 bag record command
CMD="ros2 bag record"

# Add output path
CMD="$CMD -o \"$OUTPUT_PATH\""

# Add compression options
if [ "$COMPRESSION" != "none" ]; then
    CMD="$CMD --compression-mode $COMPRESSION"
    if [ -n "$COMPRESSION_FORMAT" ]; then
        CMD="$CMD --compression-format $COMPRESSION_FORMAT"
    else
        CMD="$CMD --compression-format zstd"
    fi
fi

# Add duration limit
if [ -n "$MAX_DURATION" ]; then
    CMD="$CMD -d $MAX_DURATION"
fi

# Add size limit
if [ -n "$MAX_SIZE" ]; then
    CMD="$CMD -b $MAX_SIZE"
fi

# Add hidden topics flag
if [ "$INCLUDE_HIDDEN" = true ]; then
    CMD="$CMD --include-hidden-topics"
fi

# Add sim time flag
if [ "$USE_SIM_TIME" = true ]; then
    CMD="$CMD --use-sim-time"
fi

# Add log level
CMD="$CMD --log-level $LOG_LEVEL"

# Add topics
if [ "$RECORD_ALL" = true ]; then
    CMD="$CMD -a"
elif [ ${#CUSTOM_TOPICS[@]} -gt 0 ]; then
    # Use custom topics if provided
    for topic in "${CUSTOM_TOPICS[@]}"; do
        CMD="$CMD \"$topic\""
    done
else
    # Use default topics
    for topic in "${DEFAULT_TOPICS[@]}"; do
        CMD="$CMD \"$topic\""
    done
fi

# Display configuration
echo "=========================================="
echo "ROS2 Bag Recording Configuration"
echo "=========================================="
echo "Output:        $OUTPUT_PATH"
echo "Compression:   $COMPRESSION"
if [ -n "$MAX_DURATION" ]; then
    echo "Max Duration:  ${MAX_DURATION}s"
fi
if [ -n "$MAX_SIZE" ]; then
    echo "Max Size:      $MAX_SIZE"
fi
if [ "$RECORD_ALL" = true ]; then
    echo "Topics:        ALL"
elif [ ${#CUSTOM_TOPICS[@]} -gt 0 ]; then
    echo "Topics:        ${#CUSTOM_TOPICS[@]} custom topics"
else
    echo "Topics:        ${#DEFAULT_TOPICS[@]} default topics"
fi
echo "=========================================="
echo ""
echo "Starting recording... (Press Ctrl+C to stop)"
echo ""

# Execute the command
eval $CMD

echo ""
echo "Recording stopped. Bag saved to: $OUTPUT_PATH"
