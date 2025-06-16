#!/bin/bash

echo "=== VITA Agent Odometry Test Suite ==="
echo ""
echo "Choose test mode:"
echo "1. Test with REAL odometry data (from /rt/odom)"
echo "2. Test with SYNTHETIC odometry data (circular motion)" 
echo "3. Run both tests sequentially"
echo "4. Build and run default test"
echo ""
read -p "Enter choice (1-4): " choice

echo "Building the workspace..."
cd /root/vlm_ws
colcon build --packages-select vita_agent

echo "Sourcing the workspace..."
source install/setup.bash

case $choice in
    1)
        echo "Testing with REAL odometry data..."
        echo "Make sure your robot/simulator is running and publishing to /rt/odom"
        echo ""
        python3 /root/vlm_ws/test_odom_real.py
        ;;
    2)
        echo "Testing with SYNTHETIC odometry data..."
        echo "This will generate circular motion data for testing"
        echo ""
        python3 /root/vlm_ws/test_odom_synthetic.py
        ;;
    3)
        echo "Running both tests..."
        echo ""
        echo "First: SYNTHETIC test (10 seconds)..."
        timeout 10 python3 /root/vlm_ws/test_odom_synthetic.py
        echo ""
        echo "Now: REAL data test (make sure robot is running)..."
        python3 /root/vlm_ws/test_odom_real.py
        ;;
    4)
        echo "Running default test (with synthetic data)..."
        echo ""
        # Run the original test node
        ros2 run vita_agent just_test_odom
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
python3 src/vita_agent/vita_agent/just_test_odom.py
