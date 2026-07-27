#!/bin/bash
# View wheel_front_right LED matrix topic

set -e

echo "=================================="
echo "Viewing wheel_front_right LED Matrix"
echo "=================================="
echo ""
echo "Topic: /panel_image with frame_id='wheel_front_right'"
echo "Format: sensor_msgs/Image (8x8 RGB)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Setup Zenoh environment
export ROBOT_ID=RBXU100001
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ZENOH_CONFIG="/tmp/zenoh_view_wheel_${ROBOT_ID}.json5"
cp /home/ros2/rob_box_project/local_test/zenoh_client_config.json5 "$ZENOH_CONFIG"
sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"

# Source ROS
source /opt/ros/lyrical/setup.bash
source /home/ros2/rob_box_project/install/setup.bash

# Launch Python viewer with frame_id filter
echo "Starting filtered viewer (frame_id='wheel_front_right' only)..."
python3 /home/ros2/rob_box_project/scripts/view_wheel_front_right_filtered.py
