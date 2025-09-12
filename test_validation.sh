#!/bin/bash

# =====================================================================
# Drone Perimeter Surveillance - Complete Testing & Validation Script
# =====================================================================
# This script validates all fixes and ensures the system is working correctly

echo "🚁 Drone Perimeter Surveillance - Testing & Validation"
echo "======================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
GRAY='\033[0;37m'
NC='\033[0m' # No Color

# Navigate to workspace root
WORKSPACE_ROOT="/mnt/c/Users/Soheil/Desktop/enigma-code/drone-perimeter-surveillance"
cd "$WORKSPACE_ROOT" || exit 1

echo -e "\n${YELLOW}📋 Phase 1: Environment & Dependencies Validation${NC}"
echo "=================================================="

# Check ROS2 installation
echo -e "\n🔍 Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    ros2 --version
    echo -e "${GREEN}✅ ROS2 is installed${NC}"
else
    echo -e "${RED}❌ ROS2 not found. Please install ROS2 Jazzy${NC}"
    exit 1
fi

# Check Gazebo Harmonic
echo -e "\n🔍 Checking Gazebo Harmonic..."
if command -v gz &> /dev/null; then
    gz sim --version
    echo -e "${GREEN}✅ Gazebo Harmonic is installed${NC}"
else
    echo -e "${RED}❌ Gazebo Harmonic not found. Please install gz-harmonic${NC}"
fi

# Check required ROS2 packages
echo -e "\n🔍 Checking ROS2 packages..."
required_packages=(
    "ros_gz_sim"
    "ros_gz_bridge" 
    "gz_ros2_control"
    "robot_state_publisher"
    "xacro"
    "rviz2"
)

for package in "${required_packages[@]}"; do
    if ros2 pkg list | grep -q "$package"; then
        echo -e "${GREEN}✅ $package is available${NC}"
    else
        echo -e "${RED}❌ $package is missing${NC}"
    fi
done

echo -e "\n${YELLOW}📋 Phase 2: Build System Validation${NC}"
echo "=================================="

# Clean previous build
echo -e "\n🧹 Cleaning previous build..."
rm -rf build install log

# Source ROS2
echo -e "\n🔧 Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Build the package
echo -e "\n🔨 Building quadcopter_simulation package..."
if colcon build --packages-select quadcopter_simulation --symlink-install; then
    echo -e "${GREEN}✅ Build successful${NC}"
else
    echo -e "${RED}❌ Build failed${NC}"
    exit 1
fi

# Source the workspace
echo -e "\n🔧 Sourcing workspace..."
if source install/setup.bash; then
    echo -e "${GREEN}✅ Workspace sourced successfully${NC}"
else
    echo -e "${RED}❌ Failed to source workspace${NC}"
fi

echo -e "\n${YELLOW}📋 Phase 3: URDF Model Validation${NC}"
echo "================================"

# Check URDF syntax
echo -e "\n🔍 Validating URDF syntax..."
urdf_files=(
    "src/quadcopter_simulation/urdf/quadcopter.urdf"
    "src/quadcopter_simulation/urdf/quadcopter_simple.urdf"
)

for urdf in "${urdf_files[@]}"; do
    if [[ -f "$urdf" ]]; then
        echo "🔍 Checking $urdf..."
        if command -v check_urdf &> /dev/null; then
            if check_urdf "$urdf"; then
                echo -e "${GREEN}✅ $urdf syntax is valid${NC}"
            else
                echo -e "${RED}❌ $urdf has syntax errors${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️  check_urdf not available, skipping syntax check${NC}"
        fi
    else
        echo -e "${RED}❌ $urdf not found${NC}"
    fi
done

# Test XACRO processing
echo -e "\n🔍 Testing XACRO processing..."
if xacro src/quadcopter_simulation/urdf/quadcopter.urdf.xacro -o temp_quadcopter.urdf; then
    echo -e "${GREEN}✅ XACRO processing successful${NC}"
    rm -f temp_quadcopter.urdf
else
    echo -e "${RED}❌ XACRO processing failed${NC}"
fi

echo -e "\n${YELLOW}📋 Phase 4: File Reference Validation${NC}"
echo "===================================="

# Check that all referenced files exist
echo -e "\n🔍 Validating file references..."

declare -A file_checks=(
    ["Launch file URDF reference"]="src/quadcopter_simulation/urdf/quadcopter.urdf"
    ["Launch file world reference"]="src/quadcopter_simulation/worlds/drone_world.sdf"
    ["Launch file RViz config"]="src/quadcopter_simulation/rviz/drone_config.rviz"
    ["Controller config"]="src/quadcopter_simulation/config/drone_controllers.yaml"
    ["XACRO source file"]="src/quadcopter_simulation/urdf/quadcopter.urdf.xacro"
    ["Simple URDF file"]="src/quadcopter_simulation/urdf/quadcopter_simple.urdf"
)

for check in "${!file_checks[@]}"; do
    if [[ -f "${file_checks[$check]}" ]]; then
        echo -e "${GREEN}✅ $check: ${file_checks[$check]}${NC}"
    else
        echo -e "${RED}❌ $check: ${file_checks[$check]} - NOT FOUND${NC}"
    fi
done

echo -e "\n${YELLOW}📋 Phase 5: ROS2 System Validation${NC}"
echo "================================"

# Test package discovery
echo -e "\n🔍 Testing package discovery..."
if pkg_path=$(ros2 pkg prefix quadcopter_simulation 2>/dev/null); then
    echo -e "${GREEN}✅ quadcopter_simulation package found at: $pkg_path${NC}"
else
    echo -e "${RED}❌ quadcopter_simulation package not found${NC}"
fi

# Test executables
echo -e "\n🔍 Testing executables..."
executables=("drone_controller" "drone_teleop.py" "waypoint_follower.py" "drone_teleop_safe.py")

for exe in "${executables[@]}"; do
    if ros2 run quadcopter_simulation "$exe" --help &>/dev/null; then
        echo -e "${GREEN}✅ $exe is executable${NC}"
    else
        echo -e "${RED}❌ $exe is not executable or has issues${NC}"
    fi
done

echo -e "\n${YELLOW}📋 Phase 6: Launch File Validation${NC}"
echo "================================="

# Test launch file syntax
echo -e "\n🔍 Testing launch file syntax..."
launch_files=(
    "src/quadcopter_simulation/launch/gazebo_sim.launch.py"
    "src/quadcopter_simulation/launch/full_simulation.launch.py"
)

for launch in "${launch_files[@]}"; do
    if [[ -f "$launch" ]]; then
        if python3 -m py_compile "$launch"; then
            echo -e "${GREEN}✅ $launch syntax is valid${NC}"
        else
            echo -e "${RED}❌ $launch has syntax errors${NC}"
        fi
    else
        echo -e "${RED}❌ $launch not found${NC}"
    fi
done

echo -e "\n${YELLOW}📋 Phase 7: Integration Testing Commands${NC}"
echo "======================================="

echo -e "\n${CYAN}🚀 Manual Integration Testing Commands:${NC}"
echo "====================================="

echo -e "\n${WHITE}1. Test Basic Simulation Launch:${NC}"
echo -e "${GRAY}   ros2 launch quadcopter_simulation gazebo_sim.launch.py${NC}"

echo -e "\n${WHITE}2. Test Headless Simulation:${NC}"
echo -e "${GRAY}   ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true${NC}"

echo -e "\n${WHITE}3. Test with RViz disabled:${NC}"
echo -e "${GRAY}   ros2 launch quadcopter_simulation gazebo_sim.launch.py use_rviz:=false${NC}"

echo -e "\n${WHITE}4. Test Full Simulation (with XACRO):${NC}"
echo -e "${GRAY}   ros2 launch quadcopter_simulation full_simulation.launch.py${NC}"

echo -e "\n${WHITE}5. Test Individual Components:${NC}"
echo -e "${GRAY}   # In separate terminals:${NC}"
echo -e "${GRAY}   ros2 run quadcopter_simulation drone_controller${NC}"
echo -e "${GRAY}   ros2 run quadcopter_simulation drone_teleop.py${NC}"
echo -e "${GRAY}   ros2 run quadcopter_simulation waypoint_follower.py${NC}"

echo -e "\n${WHITE}6. Monitor System Health:${NC}"
echo -e "${GRAY}   ros2 node list${NC}"
echo -e "${GRAY}   ros2 topic list${NC}"
echo -e "${GRAY}   ros2 topic hz /cmd_vel${NC}"
echo -e "${GRAY}   ros2 topic echo /drone/imu/data${NC}"

echo -e "\n${WHITE}7. Debug Commands:${NC}"
echo -e "${GRAY}   ros2 doctor${NC}"
echo -e "${GRAY}   ros2 wtf${NC}"
echo -e "${GRAY}   gz topic -l${NC}"
echo -e "${GRAY}   gz model -l${NC}"

echo -e "\n${YELLOW}📋 Phase 8: Performance Testing${NC}"
echo "=============================="

echo -e "\n🔍 Performance monitoring commands:"
echo "   # System resource usage:"
echo "   htop"
echo "   # Topic rates:"
echo "   ros2 topic hz /drone/imu/data"
echo "   ros2 topic hz /drone/camera/image_raw"

echo -e "\n${YELLOW}📋 Testing Summary${NC}"
echo "=================="

echo -e "\n${CYAN}🎯 Key Success Criteria:${NC}"
echo "✓ All builds complete without errors"
echo "✓ All file references resolve correctly"
echo "✓ URDF models load without warnings"
echo "✓ Gazebo simulation starts successfully"
echo "✓ ROS2 topics are published correctly"
echo "✓ Control interfaces respond to commands"

echo -e "\n${RED}🚨 Known Issues to Watch For:${NC}"
echo "• Clock skew warnings in WSL (run: sudo hwclock -s)"
echo "• Python script permissions (run: dos2unix + chmod +x)"
echo "• Gazebo plugin loading errors"
echo "• Missing ROS2 bridge connections"

echo -e "\n${GREEN}✅ Validation script completed!${NC}"
echo "Run the manual integration tests above to verify full functionality."