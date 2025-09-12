# =====================================================================
# Drone Perimeter Surveillance - Complete Testing & Validation Script
# =====================================================================
# This script validates all fixes and ensures the system is working correctly

Write-Host "🚁 Drone Perimeter Surveillance - Testing & Validation" -ForegroundColor Cyan
Write-Host "=======================================================" -ForegroundColor Cyan

# Set error handling
$ErrorActionPreference = "Continue"

# Navigate to workspace root
$WORKSPACE_ROOT = "C:\Users\Soheil\Desktop\enigma-code\drone-perimeter-surveillance"
Set-Location $WORKSPACE_ROOT

Write-Host "`n📋 Phase 1: Environment & Dependencies Validation" -ForegroundColor Yellow
Write-Host "=================================================="

# Check ROS2 installation
Write-Host "`n🔍 Checking ROS2 installation..."
try {
    ros2 --version
    Write-Host "✅ ROS2 is installed" -ForegroundColor Green
} catch {
    Write-Host "❌ ROS2 not found. Please install ROS2 Jazzy" -ForegroundColor Red
    exit 1
}

# Check Gazebo Harmonic
Write-Host "`n🔍 Checking Gazebo Harmonic..."
try {
    gz sim --version
    Write-Host "✅ Gazebo Harmonic is installed" -ForegroundColor Green
} catch {
    Write-Host "❌ Gazebo Harmonic not found. Please install gz-harmonic" -ForegroundColor Red
}

# Check required ROS2 packages
Write-Host "`n🔍 Checking ROS2 packages..."
$required_packages = @(
    "ros_gz_sim",
    "ros_gz_bridge", 
    "gz_ros2_control",
    "robot_state_publisher",
    "xacro",
    "rviz2"
)

foreach ($package in $required_packages) {
    try {
        ros2 pkg list | Select-String $package | Out-Null
        if ($?) {
            Write-Host "✅ $package is available" -ForegroundColor Green
        } else {
            Write-Host "❌ $package is missing" -ForegroundColor Red
        }
    } catch {
        Write-Host "❌ Error checking $package" -ForegroundColor Red
    }
}

Write-Host "`n📋 Phase 2: Build System Validation" -ForegroundColor Yellow
Write-Host "=================================="

# Clean previous build
Write-Host "`n🧹 Cleaning previous build..."
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "install") { Remove-Item -Recurse -Force "install" }
if (Test-Path "log") { Remove-Item -Recurse -Force "log" }

# Source ROS2
Write-Host "`n🔧 Sourcing ROS2 environment..."
# Note: In PowerShell, we need to call setup scripts differently
& "C:\opt\ros\jazzy\setup.ps1" -ErrorAction SilentlyContinue

# Build the package
Write-Host "`n🔨 Building quadcopter_simulation package..."
try {
    colcon build --packages-select quadcopter_simulation --symlink-install
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Build successful" -ForegroundColor Green
    } else {
        Write-Host "❌ Build failed with exit code $LASTEXITCODE" -ForegroundColor Red
    }
} catch {
    Write-Host "❌ Build command failed: $_" -ForegroundColor Red
}

# Source the workspace
Write-Host "`n🔧 Sourcing workspace..."
try {
    & ".\install\setup.ps1"
    Write-Host "✅ Workspace sourced successfully" -ForegroundColor Green
} catch {
    Write-Host "❌ Failed to source workspace" -ForegroundColor Red
}

Write-Host "`n📋 Phase 3: URDF Model Validation" -ForegroundColor Yellow
Write-Host "================================"

# Check URDF syntax
Write-Host "`n🔍 Validating URDF syntax..."
$urdf_files = @(
    "src\quadcopter_simulation\urdf\quadcopter.urdf",
    "src\quadcopter_simulation\urdf\quadcopter_simple.urdf"
)

foreach ($urdf in $urdf_files) {
    if (Test-Path $urdf) {
        Write-Host "🔍 Checking $urdf..."
        try {
            check_urdf $urdf
            if ($LASTEXITCODE -eq 0) {
                Write-Host "✅ $urdf syntax is valid" -ForegroundColor Green
            } else {
                Write-Host "❌ $urdf has syntax errors" -ForegroundColor Red
            }
        } catch {
            Write-Host "⚠️  check_urdf not available, skipping syntax check" -ForegroundColor Yellow
        }
    } else {
        Write-Host "❌ $urdf not found" -ForegroundColor Red
    }
}

# Test XACRO processing
Write-Host "`n🔍 Testing XACRO processing..."
try {
    xacro src\quadcopter_simulation\urdf\quadcopter.urdf.xacro -o temp_quadcopter.urdf
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ XACRO processing successful" -ForegroundColor Green
        Remove-Item "temp_quadcopter.urdf" -ErrorAction SilentlyContinue
    } else {
        Write-Host "❌ XACRO processing failed" -ForegroundColor Red
    }
} catch {
    Write-Host "❌ XACRO command failed: $_" -ForegroundColor Red
}

Write-Host "`n📋 Phase 4: File Reference Validation" -ForegroundColor Yellow
Write-Host "===================================="

# Check that all referenced files exist
Write-Host "`n🔍 Validating file references..."

$file_checks = @{
    "Launch file URDF reference" = "src\quadcopter_simulation\urdf\quadcopter.urdf"
    "Launch file world reference" = "src\quadcopter_simulation\worlds\drone_world.sdf"
    "Launch file RViz config" = "src\quadcopter_simulation\rviz\drone_config.rviz"
    "Controller config" = "src\quadcopter_simulation\config\drone_controllers.yaml"
    "XACRO source file" = "src\quadcopter_simulation\urdf\quadcopter.urdf.xacro"
    "Simple URDF file" = "src\quadcopter_simulation\urdf\quadcopter_simple.urdf"
}

foreach ($check in $file_checks.GetEnumerator()) {
    if (Test-Path $check.Value) {
        Write-Host "✅ $($check.Key): $($check.Value)" -ForegroundColor Green
    } else {
        Write-Host "❌ $($check.Key): $($check.Value) - NOT FOUND" -ForegroundColor Red
    }
}

Write-Host "`n📋 Phase 5: ROS2 System Validation" -ForegroundColor Yellow
Write-Host "================================"

# Test package discovery
Write-Host "`n🔍 Testing package discovery..."
try {
    $pkg_path = ros2 pkg prefix quadcopter_simulation
    if ($pkg_path) {
        Write-Host "✅ quadcopter_simulation package found at: $pkg_path" -ForegroundColor Green
    } else {
        Write-Host "❌ quadcopter_simulation package not found" -ForegroundColor Red
    }
} catch {
    Write-Host "❌ Package discovery failed" -ForegroundColor Red
}

# Test executables
Write-Host "`n🔍 Testing executables..."
$executables = @("drone_controller", "drone_teleop.py", "waypoint_follower.py", "drone_teleop_safe.py")

foreach ($exe in $executables) {
    try {
        ros2 run quadcopter_simulation $exe --help 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ $exe is executable" -ForegroundColor Green
        } else {
            Write-Host "❌ $exe is not executable or has issues" -ForegroundColor Red
        }
    } catch {
        Write-Host "⚠️  Could not test $exe" -ForegroundColor Yellow
    }
}

Write-Host "`n📋 Phase 6: Launch File Validation" -ForegroundColor Yellow
Write-Host "================================="

# Test launch file syntax
Write-Host "`n🔍 Testing launch file syntax..."
$launch_files = @(
    "src\quadcopter_simulation\launch\gazebo_sim.launch.py",
    "src\quadcopter_simulation\launch\full_simulation.launch.py"
)

foreach ($launch in $launch_files) {
    if (Test-Path $launch) {
        try {
            # Test Python syntax
            python -m py_compile $launch
            if ($LASTEXITCODE -eq 0) {
                Write-Host "✅ $launch syntax is valid" -ForegroundColor Green
            } else {
                Write-Host "❌ $launch has syntax errors" -ForegroundColor Red
            }
        } catch {
            Write-Host "❌ Failed to validate $launch" -ForegroundColor Red
        }
    } else {
        Write-Host "❌ $launch not found" -ForegroundColor Red
    }
}

Write-Host "`n📋 Phase 7: Integration Testing Commands" -ForegroundColor Yellow
Write-Host "======================================="

Write-Host "`n🚀 Manual Integration Testing Commands:" -ForegroundColor Cyan
Write-Host "=====================================`n"

Write-Host "1. Test Basic Simulation Launch:" -ForegroundColor White
Write-Host "   ros2 launch quadcopter_simulation gazebo_sim.launch.py" -ForegroundColor Gray

Write-Host "`n2. Test Headless Simulation:" -ForegroundColor White
Write-Host "   ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true" -ForegroundColor Gray

Write-Host "`n3. Test with RViz disabled:" -ForegroundColor White
Write-Host "   ros2 launch quadcopter_simulation gazebo_sim.launch.py use_rviz:=false" -ForegroundColor Gray

Write-Host "`n4. Test Full Simulation (with XACRO):" -ForegroundColor White
Write-Host "   ros2 launch quadcopter_simulation full_simulation.launch.py" -ForegroundColor Gray

Write-Host "`n5. Test Individual Components:" -ForegroundColor White
Write-Host "   # In separate terminals:" -ForegroundColor Gray
Write-Host "   ros2 run quadcopter_simulation drone_controller" -ForegroundColor Gray
Write-Host "   ros2 run quadcopter_simulation drone_teleop.py" -ForegroundColor Gray
Write-Host "   ros2 run quadcopter_simulation waypoint_follower.py" -ForegroundColor Gray

Write-Host "`n6. Monitor System Health:" -ForegroundColor White
Write-Host "   ros2 node list" -ForegroundColor Gray
Write-Host "   ros2 topic list" -ForegroundColor Gray
Write-Host "   ros2 topic hz /cmd_vel" -ForegroundColor Gray
Write-Host "   ros2 topic echo /drone/imu/data" -ForegroundColor Gray

Write-Host "`n7. Debug Commands:" -ForegroundColor White
Write-Host "   ros2 doctor" -ForegroundColor Gray
Write-Host "   ros2 wtf" -ForegroundColor Gray
Write-Host "   gz topic -l" -ForegroundColor Gray
Write-Host "   gz model -l" -ForegroundColor Gray

Write-Host "`n📋 Phase 8: Performance Testing" -ForegroundColor Yellow
Write-Host "=============================="

Write-Host "`n🔍 Performance monitoring commands:"
Write-Host "   # CPU/Memory usage:"
Write-Host "   Get-Process -Name gz,rviz2,robot_state_publisher | Format-Table Name,CPU,WorkingSet"
Write-Host "   # Topic rates:"
Write-Host "   ros2 topic hz /drone/imu/data"
Write-Host "   ros2 topic hz /drone/camera/image_raw"

Write-Host "`n📋 Testing Summary" -ForegroundColor Yellow
Write-Host "=================="

Write-Host "`n🎯 Key Success Criteria:" -ForegroundColor Cyan
Write-Host "✓ All builds complete without errors"
Write-Host "✓ All file references resolve correctly"
Write-Host "✓ URDF models load without warnings"
Write-Host "✓ Gazebo simulation starts successfully"
Write-Host "✓ ROS2 topics are published correctly"
Write-Host "✓ Control interfaces respond to commands"

Write-Host "`n🚨 Known Issues to Watch For:" -ForegroundColor Red
Write-Host "• Clock skew warnings in WSL (run: sudo hwclock -s)"
Write-Host "• Python script permissions (run: dos2unix + chmod +x)"
Write-Host "• Gazebo plugin loading errors"
Write-Host "• Missing ROS2 bridge connections"

Write-Host "`n✅ Validation script completed!" -ForegroundColor Green
Write-Host "Run the manual integration tests above to verify full functionality.`n"