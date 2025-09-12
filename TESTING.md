# Testing & Validation Guide

This directory contains comprehensive testing scripts to validate all fixes and ensure the drone simulation system works correctly.

## 🚀 Quick Start Testing

### For Windows (PowerShell):
```powershell
# Run quick test
.\quick_test.bat

# Run comprehensive validation
PowerShell -ExecutionPolicy Bypass -File .\test_validation.ps1
```

### For Linux/WSL:
```bash
# Make scripts executable
chmod +x *.sh

# Run quick test
./quick_test.sh

# Run comprehensive validation
./test_validation.sh
```

## 📋 Testing Scripts Overview

### 1. `quick_test.sh/.bat`
**Purpose**: Fast validation of critical fixes
**Runtime**: ~2-3 minutes
**Tests**:
- ✅ Build system functionality
- ✅ Package discovery
- ✅ XACRO processing
- ✅ Launch file syntax

### 2. `test_validation.sh/.ps1`
**Purpose**: Comprehensive system validation
**Runtime**: ~5-10 minutes
**Tests**:
- ✅ Environment & dependencies
- ✅ Build system validation
- ✅ URDF model validation
- ✅ File reference validation
- ✅ ROS2 system validation
- ✅ Launch file validation
- ✅ Integration testing commands

## 🎯 Manual Integration Tests

After running the validation scripts, perform these manual tests:

### Test 1: Basic Simulation
```bash
ros2 launch quadcopter_simulation gazebo_sim.launch.py
```
**Expected**: Gazebo opens with quadcopter model loaded

### Test 2: Headless Mode
```bash
ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true
```
**Expected**: Simulation runs without GUI

### Test 3: Individual Components
```bash
# Terminal 1: Controller
ros2 run quadcopter_simulation drone_controller

# Terminal 2: Teleoperation
ros2 run quadcopter_simulation drone_teleop.py

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel
```

## 🔍 System Health Monitoring

### Check Active Nodes:
```bash
ros2 node list
```

### Monitor Topic Rates:
```bash
ros2 topic hz /drone/imu/data
ros2 topic hz /cmd_vel
```

### Check Gazebo Status:
```bash
gz topic -l
gz model -l
```

## ✅ Success Criteria

Your system is working correctly if:

1. **Build Success**: `colcon build` completes without errors
2. **Package Discovery**: `ros2 pkg prefix quadcopter_simulation` returns a path
3. **XACRO Processing**: No errors when processing URDF files
4. **Launch Success**: Gazebo starts and loads the drone model
5. **Topic Publishing**: IMU, camera, and control topics are active
6. **Control Response**: Drone responds to teleop commands

## 🚨 Common Issues & Solutions

### Issue: "Package not found"
**Solution**: 
```bash
source install/setup.bash
```

### Issue: "Clock skew" warnings (WSL)
**Solution**:
```bash
sudo hwclock -s
```

### Issue: Python script permissions
**Solution**:
```bash
chmod +x src/quadcopter_simulation/scripts/*.py
dos2unix src/quadcopter_simulation/scripts/*.py
```

### Issue: Gazebo plugin errors
**Solution**: Check that all plugins are using modern Gazebo Harmonic syntax

## 📊 Performance Benchmarks

### Expected Performance:
- **Build Time**: < 30 seconds
- **Launch Time**: < 10 seconds
- **IMU Rate**: ~250 Hz
- **Control Latency**: < 10ms
- **Memory Usage**: < 1GB

### Monitor Performance:
```bash
# CPU/Memory usage
htop

# Topic rates
ros2 topic hz /drone/imu/data

# System diagnostics
ros2 doctor
```

## 🔧 Debug Commands

### ROS2 Debugging:
```bash
ros2 doctor           # System health check
ros2 wtf              # Detailed diagnostics
ros2 bag record -a    # Record all topics
```

### Gazebo Debugging:
```bash
gz sim --verbose 4    # Verbose logging
gz topic -l           # List topics
gz model -l           # List models
```

## 📈 Test Results Log

Create a test log with results:

```bash
echo "Test Results - $(date)" > test_results.log
./test_validation.sh >> test_results.log 2>&1
```

## 🎉 Next Steps

After successful validation:

1. **Implement Advanced Features**:
   - EKF localization
   - Obstacle avoidance
   - Computer vision

2. **Performance Optimization**:
   - PID tuning
   - Sensor fusion improvements
   - Real-time constraints

3. **Production Deployment**:
   - Docker containerization
   - CI/CD pipeline
   - Documentation updates

---

**Need Help?** Check the main README.md for troubleshooting and contact information.