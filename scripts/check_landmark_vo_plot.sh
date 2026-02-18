#!/bin/bash
# Check if /landmark_vo_plot_2d/landmark_vo_plot_image is being updated
# Run: ./scripts/check_landmark_vo_plot.sh

TOPIC="/landmark_vo_plot_2d/landmark_vo_plot_image"
echo "Checking topic: $TOPIC"
echo ""

# Check if topic exists
if ! ros2 topic list | grep -q "landmark_vo_plot_image"; then
    echo "Topic not found. Is landmark_vo_plot_2d node running?"
    echo "  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py"
    exit 1
fi

# Check publish rate (run for 5 seconds)
echo "Publish rate (5 second sample):"
ros2 topic hz "$TOPIC" --window 5 2>/dev/null || echo "No messages received."

echo ""
echo "Message type:"
ros2 topic info "$TOPIC" -v 2>/dev/null || true
