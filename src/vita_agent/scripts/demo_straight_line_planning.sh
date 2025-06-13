#!/bin/bash

# Demo script for straight-line path planning feature
# This script demonstrates how to launch the vita_agent with straight-line planning enabled

echo "=== VITA Agent Straight-Line Path Planning Demo ==="
echo

# Check if we're in the right workspace
if [ ! -d "/home/heng.li/repo/vlm_ws/src/vita_agent" ]; then
    echo "Error: Not in the correct workspace directory"
    exit 1
fi

# Source ROS2 environment
echo "1. Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source /home/heng.li/repo/vlm_ws/install/setup.bash

echo "2. Available launch options:"
echo
echo "Option A: Launch with straight-line planning enabled"
echo "   ros2 run vita_agent zero_shot_vlm_planner --ros-args -p use_straight_line_planning:=true"
echo
echo "Option B: Launch with configuration file"
echo "   ros2 run vita_agent zero_shot_vlm_planner --ros-args --params-file /home/heng.li/repo/vlm_ws/src/vita_agent/config/straight_line_planning.yaml"
echo
echo "Option C: Launch with default settings (obstacle avoidance enabled)"
echo "   ros2 run vita_agent zero_shot_vlm_planner"
echo

echo "3. Runtime parameter control:"
echo "   # Enable straight-line planning"
echo "   ros2 param set /task_logic_node use_straight_line_planning true"
echo
echo "   # Disable straight-line planning"
echo "   ros2 param set /task_logic_node use_straight_line_planning false"
echo
echo "   # Check current setting"
echo "   ros2 param get /task_logic_node use_straight_line_planning"
echo

echo "4. Safety reminders:"
echo "   ⚠️  Only use straight-line planning in obstacle-free environments"
echo "   ⚠️  Maintain visual supervision when enabled"
echo "   ⚠️  Test in controlled environments first"
echo

read -p "Press Enter to continue or Ctrl+C to exit..."

echo "5. Testing the implementation..."
cd /home/heng.li/repo/vlm_ws/src/vita_agent
python3 test_straight_line_planning.py

echo
echo "✅ Demo completed successfully!"
echo "The straight-line path planning feature is ready to use."
