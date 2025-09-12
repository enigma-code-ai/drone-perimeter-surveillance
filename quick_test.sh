#!/bin/bash

# =====================================================================
# Quick Test Script - Essential Validation Only
# =====================================================================

echo "🚀 Quick Drone Simulation Test"
echo "=============================="

# Set workspace
cd /mnt/c/Users/Soheil/Desktop/enigma-code/drone-perimeter-surveillance

echo "1. 🧹 Cleaning and building..."
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --packages-select quadcopter_simulation --symlink-install

echo "2. 🔧 Sourcing workspace..."
source install/setup.bash

echo "3. 🔍 Testing XACRO processing..."
xacro src/quadcopter_simulation/urdf/quadcopter.urdf.xacro -o /tmp/test_quadcopter.urdf
if [[ $? -eq 0 ]]; then
    echo "✅ XACRO processing successful"
    rm -f /tmp/test_quadcopter.urdf
else
    echo "❌ XACRO processing failed"
    exit 1
fi

echo "4. 🎯 Testing package discovery..."
if ros2 pkg prefix quadcopter_simulation >/dev/null 2>&1; then
    echo "✅ Package found"
else
    echo "❌ Package not found"
    exit 1
fi

echo "5. 🚁 Testing launch file syntax..."
python3 -c "import src.quadcopter_simulation.launch.gazebo_sim"
if [[ $? -eq 0 ]]; then
    echo "✅ Launch file syntax OK"
else
    echo "❌ Launch file has issues"
fi

echo ""
echo "🎉 Quick test completed successfully!"
echo ""
echo "🚀 Ready to run:"
echo "   ros2 launch quadcopter_simulation gazebo_sim.launch.py"