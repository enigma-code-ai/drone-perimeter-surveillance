@echo off
REM =====================================================================
REM Quick Windows Test Script - Essential Validation Only
REM =====================================================================

echo 🚀 Quick Drone Simulation Test
echo ==============================

cd /d C:\Users\Soheil\Desktop\enigma-code\drone-perimeter-surveillance

echo 1. 🧹 Cleaning previous build...
if exist build rmdir /s /q build
if exist install rmdir /s /q install  
if exist log rmdir /s /q log

echo 2. 🔧 Sourcing ROS2...
call C:\opt\ros\jazzy\setup.bat

echo 3. 🔨 Building package...
colcon build --packages-select quadcopter_simulation --symlink-install
if %errorlevel% neq 0 (
    echo ❌ Build failed
    pause
    exit /b 1
)

echo 4. 🔧 Sourcing workspace...
call install\setup.bat

echo 5. 🎯 Testing package discovery...
ros2 pkg prefix quadcopter_simulation >nul 2>&1
if %errorlevel% equ 0 (
    echo ✅ Package found
) else (
    echo ❌ Package not found
    pause
    exit /b 1
)

echo.
echo 🎉 Quick test completed successfully!
echo.
echo 🚀 Ready to run:
echo    ros2 launch quadcopter_simulation gazebo_sim.launch.py
echo.
pause