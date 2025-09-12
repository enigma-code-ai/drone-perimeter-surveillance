
# Drone Perimeter Surveillance System

A comprehensive autonomous drone system for perimeter surveillance featuring AI-powered threat detection, autonomous navigation, and real-time monitoring capabilities. Built on ROS2 Jazzy and Gazebo Harmonic with modern simulation and control systems.

## 🚁 Business Overview

### Problem Statement
Development of autonomous drone systems for perimeter surveillance addressing critical security needs across residential, commercial, and industrial sectors.

### Core Capabilities
- **Autonomous Perimeter Patrols**: Zone-based monitoring with intelligent route planning
- **Threat Detection**: AI-powered detection of intruders, unauthorized vehicles, wildlife encroachments, environmental hazards (fires, leaks), and weapons
- **Alert Integration**: Real-time alerts with human security team coordination
- **Counter-Drone Features**: Detection and response to rogue drones
- **Automated Deterrence**: Non-lethal responses including lights, sirens, and verbal warnings
- **Evidence Generation**: Automated logs with HD video evidence for insurance and legal purposes

### Market Opportunity
- **Global Market**: $7.2 billion surveillance drone market growing at 14.4% CAGR
- **Target Segments**: Residential ($150-$250 per incident), Commercial ($500-$1,500 monthly), Enterprise ($20K+ advanced setups)
- **Service Model**: On-demand deployment, subscription monitoring, enterprise contracts

## 🛠️ Technical Implementation
### System Features

- 🚁 **Realistic Quadcopter Dynamics**: Full 6-DOF drone model with accurate physics simulation
- 🎮 **Multiple Control Modes**: 
  - Manual teleoperation via keyboard
  - Autonomous waypoint navigation
  - Position hold and stabilization
- 📡 **Sensor Suite**:
  - IMU (Inertial Measurement Unit)
  - HD Camera with real-time streaming
  - GPS positioning (simulated)
- 🎯 **Advanced Control**:
  - Cascaded PID controllers (position, attitude, rate)
  - Motor mixing for X-configuration quadcopter
  - Automatic takeoff and landing sequences
- 🌍 **3D Visualization**: 
  - Real-time simulation in Gazebo Harmonic
  - RViz integration for sensor data and trajectory visualization
- 🔧 **Modular Architecture**: Easy to extend for different drone configurations

## 🚀 Quick Start

### Prerequisites
- Ubuntu 24.04 LTS (Noble)
- ROS2 Jazzy Jalisco
- Gazebo Harmonic
- 8GB+ RAM recommended

### Installation

```bash
# Navigate to project directory
cd /mnt/c/Users/Soheil/Desktop/enigma-code/drone-perimeter-surveillance

# Clean previous builds
rm -rf build install log

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Fix script permissions (if needed)
chmod +x src/quadcopter_simulation/scripts/*.py
dos2unix src/quadcopter_simulation/scripts/*.py
```

### Launch Simulation

```bash
# Launch complete simulation with GUI
ros2 launch quadcopter_simulation gazebo_sim.launch.py

# Launch headless for testing
ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true

# In separate terminal, run professional teleop system
ros2 run quadcopter_simulation drone_teleop.py
```

## 🎮 Professional Teleoperation System

Our advanced teleoperation system includes:

### 🎛️ **Multiple Flight Modes**
- **Manual**: Direct control for experienced operators
- **Stabilized**: Attitude stabilization with position control
- **Altitude Hold**: Automatic altitude maintenance
- **Position Hold**: Full 3D position and altitude hold
- **Auto Mission**: Autonomous waypoint navigation
- **Return-to-Home**: Automatic return to launch position

### 🛡️ **Advanced Safety Systems**
- Real-time geofencing with configurable boundaries
- Battery monitoring with critical alerts
- Distance and altitude limiting
- Emergency failsafe systems
- Multi-level safety status monitoring

### 📊 **Real-Time HUD Interface**
- Live telemetry display (position, velocity, orientation)
- Flight status indicators and safety alerts
- Mission progress tracking
- Performance metrics and analytics
- Professional terminal-based interface

### Control Interface
- **Flight Modes**: M (Manual), S (Stabilized), A (Altitude Hold), P (Position Hold), G (Auto Mission), H (Return Home)
- **Movement**: W/A/S/D (Move), Q/E (Rotate), R/F (Up/Down)
- **Commands**: T (Takeoff), L (Land), Space (ARM/DISARM), Enter (Emergency Stop)
- **Mission**: N (Add Waypoint), B (Begin Mission), C (Clear Mission)
- **System**: I (Info), +/- (Speed), ESC (Emergency), Ctrl+C (Quit)

## 🎮 Teleoperation Controls

| Key | Action |
|-----|--------|
| W/S | Forward/Backward |
| A/D | Left/Right |
| Q/E | Rotate Left/Right |
| R/F | Up/Down |
| T | Takeoff (auto hover at 1m) |
| L | Land |
| Space | ARM/DISARM |
| H | Hold position |
| 1/2 | Decrease/Increase linear speed |
| 3/4 | Decrease/Increase angular speed |
| ESC | Emergency Stop |
| Ctrl+C | Quit |

## Project Structure

```
drone-perimeter-surveillance/
├── src/
│   ├── quadcopter_simulation/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   ├── drone_controllers.yaml    # PID controller configurations
│   │   │   └── ekf_localization.yaml     # Sensor fusion parameters
│   │   ├── launch/
│   │   │   └── gazebo_sim.launch.py      # Main launch file
│   │   ├── urdf/
│   │   │   ├── quadcopter.urdf.xacro     # Complete drone model (source)
│   │   │   ├── quadcopter.urdf           # Compiled drone model
│   │   │   └── quadcopter_simple.urdf    # Simplified model for testing
│   │   ├── rviz/
│   │   │   └── drone_config.rviz         # RViz visualization config
│   │   ├── worlds/
│   │   │   └── drone_world.sdf           # Gazebo world definition
│   │   ├── src/
│   │   │   └── drone_controller.cpp      # Main control implementation
│   │   └── scripts/
│   │       ├── drone_teleop.py           # Keyboard teleoperation
│   │       └── waypoint_follower.py      # Autonomous navigation
│   └── drone/                             # Additional custom packages
├── build/                                 # Build artifacts (generated)
├── install/                               # Install space (generated)
├── log/                                   # Build logs (generated)
└── README.md
```

## 🧪 Testing & Validation

### System Validation
```bash
# Verify installation
ros2 doctor

# Test build system
colcon build --symlink-install

# Validate URDF syntax
check_urdf src/quadcopter_simulation/urdf/quadcopter.urdf

# Test simulation launch
ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true &
sleep 10
pkill -f gazebo

# Verify ROS2 nodes
ros2 node list
ros2 topic list
```

### Integration Testing
```bash
# Test individual components
ros2 run quadcopter_simulation drone_controller
ros2 run quadcopter_simulation drone_teleop.py
ros2 run quadcopter_simulation waypoint_follower.py

# Monitor system performance
ros2 topic hz /cmd_vel
ros2 topic echo /drone/pose
```

## 🏗️ Architecture

### Control Architecture
```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐
│   Teleop/   │────▶│   Position   │────▶│   Attitude   │
│  Waypoint   │     │  Controller  │     │  Controller  │
└─────────────┘     └──────────────┘     └──────────────┘
                            │                     │
                            ▼                     ▼
                    ┌──────────────┐     ┌──────────────┐
                    │     Rate     │────▶│    Motor     │
                    │  Controller  │     │    Mixer     │
                    └──────────────┘     └──────────────┘
                                                 │
                                                 ▼
                                         ┌──────────────┐
                                         │   Gazebo     │
                                         │  Simulation  │
                                         └──────────────┘
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/drone/imu/data` | `sensor_msgs/Imu` | IMU sensor data |
| `/drone/pose` | `geometry_msgs/PoseStamped` | Current drone pose |
| `/drone/camera/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/motor_commands` | `std_msgs/Float64MultiArray` | Motor speed commands |
| `/flight_mode` | `std_msgs/String` | Current flight mode |
| `/arm` | `std_msgs/Bool` | Arm/Disarm status |

## Configuration

### PID Tuning

Edit `config/drone_controllers.yaml` to adjust PID gains:

```yaml
position_z_pid:
  kp: 2.0    # Proportional gain
  ki: 0.1    # Integral gain
  kd: 1.0    # Derivative gain
```

### Drone Physical Parameters

Modify drone properties in the URDF file:
- Mass: 1.5 kg (default)
- Arm length: 0.22 m
- Motor constants based on 2300KV brushless motors

## Troubleshooting

### Common Issues

**1. Clock skew warnings in WSL:**
```bash
sudo hwclock -s
```

**2. Python scripts fail with "No such file or directory":**
```bash
# Fix line endings
dos2unix src/quadcopter_simulation/scripts/*.py
chmod +x src/quadcopter_simulation/scripts/*.py
```

**3. Gazebo doesn't start:**
```bash
# Test Gazebo standalone
gz sim -v4 empty.sdf

# Check installation
dpkg -l | grep gz-harmonic
```

**4. Cannot find package during build:**
```bash
# Ensure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Clean rebuild
rm -rf build install log
colcon build --symlink-install
```

### Performance Optimization

- Reduce Gazebo physics update rate if experiencing lag
- Disable camera plugin if not needed
- Use `quadcopter_simple.urdf` for basic testing

## 📈 Development Roadmap

### Phase 1: Research & Planning (Months 0-2)
- **MVP Definition**: 3 prioritized use cases (perimeter patrol, event monitoring, counter-drone)
- **Market Analysis**: Competitor mapping, regulatory requirements (EASA/FAA/EU)
- **Technical Stack**: ROS2, PX4, Gazebo/Ignition, YOLOv8/TensorRT, Jetson platform
- **KPIs**: Localization RMSE, detection precision/recall, latency, endurance

### Phase 2: Mechanical & Systems Design (Months 2-5)
- **CAD Development**: Modular payload bays, ≤5kg envelope, FEA analysis
- **URDF Integration**: Export from SolidWorks, validate kinematics
- **Thermal Management**: Companion computer and jamming hardware cooling
- **Supply Chain**: Dual-source components, lead time optimization

### Phase 3: Core Software & Autonomy (Months 4-10)
- **Flight Control**: PX4 integration, PID/LQR/MPC trajectory tracking
- **State Estimation**: EKF/UKF sensor fusion (IMU, GNSS RTK, VIO, LiDAR)
- **SLAM Implementation**: RTAB-Map/Cartographer for persistent mapping
- **Path Planning**: A*/D* global + MPC/RRT* local planning with obstacle avoidance
- **AI Pipeline**: YOLOv8 training, model optimization, active learning
- **Mission Management**: State machine execution, fail-safes, geofencing

### Phase 4: Integration & Security (Months 8-14)
- **Communication Stack**: GStreamer low-latency streaming, DTLS/mTLS encryption
- **Security Framework**: TPM integration, secure boot, signed firmware
- **Testing & Validation**: SITL simulation, hardware-in-the-loop, field testing
- **Regulatory Compliance**: Certification preparation, documentation

### Future Enhancements
- Multi-drone swarm coordination
- Advanced computer vision (object tracking, landing pad detection)
- Weather resistance and wind disturbance handling
- Battery optimization and hot-swap capabilities
- Edge AI processing with real-time inference

## Testing

### Unit Tests
```bash
colcon test --packages-select quadcopter_simulation
colcon test-result --verbose
```

### Integration Testing
```bash
# Test individual nodes
ros2 run quadcopter_simulation drone_controller
ros2 run quadcopter_simulation drone_teleop.py

# Monitor system
ros2 node list
ros2 topic list
ros2 topic hz /cmd_vel
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Coding Standards
- Follow ROS2 coding conventions
- Use meaningful variable names
- Comment complex algorithms
- Add unit tests for new features

## 🚀 Future Development

- [ ] GPS waypoint navigation with RTK precision
- [ ] LiDAR-based obstacle avoidance and SLAM
- [ ] Multi-drone swarm coordination protocols
- [ ] AI-powered computer vision for threat detection
- [ ] Precision landing with visual markers
- [ ] Battery simulation and hot-swap management
- [ ] Advanced weather and wind disturbance modeling
- [ ] Real-time path planning (RRT*, A*, D* Lite)
- [ ] Counter-drone jamming and neutralization systems
- [ ] Edge AI processing with TensorRT optimization

## 📄 License & Contact

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

**Project Maintainer**: Drone Perimeter Surveillance Team  
**Repository**: [drone-perimeter-surveillance](https://github.com/yourusername/drone-perimeter-surveillance)

### Citation
```bibtex
@software{drone_perimeter_surveillance_2025,
  author = {Drone Surveillance Team},
  title = {Autonomous Drone Perimeter Surveillance System},
  year = {2025},
  url = {https://github.com/yourusername/drone-perimeter-surveillance}
}
```