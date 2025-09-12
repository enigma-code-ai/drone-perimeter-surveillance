
**Problem Statement**: What is the core issue the client is facing?
Autonomous drone systems for perimeter surveillance.

**Scope Boundaries**: What is included and excluded from this problem?
- Location and zone patrols
- Thread detection (e.g., intruders, unauthorized vehicles, wildlife encroachments, or environmental hazards like fires or leaks, weapon detection, etc.). 
- Alert system and integration with human security teams.
- Counter-drone features (for rogue drones)
- Automated deterrence: Drones could deploy non-lethal responses like bright lights, sirens, or verbal warnings via speakers.
- Generate automated logs with video evidence for insurance or legal purposes.


cd /mnt/c/Users/Soheil/Desktop/enigma-code/drone-perimeter-surveillance

# Clean build
rm -rf build install log

# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Fix line endings if scripts fail with "No such file or directory"
chmod +x src/quadcopter_simulation/scripts/*.py

# Fix line endings if needed
dos2unix src/quadcopter_simulation/scripts/*.py

# Launch with the new configuration
ros2 launch quadcopter_simulation gazebo_sim.launch.py

# Or launch headless (no GUI) for testing
ros2 launch quadcopter_simulation gazebo_sim.launch.py headless:=true

# In a separate terminal, run the safe teleop
ros2 run quadcopter_simulation drone_teleop_safe.py













Here's a comprehensive README file for your quadcopter drone simulation project:

```markdown
# ROS2 Quadcopter Drone Simulation

A complete ROS2-based quadcopter drone simulation package featuring realistic physics, autonomous navigation, and teleoperation capabilities using Gazebo Harmonic and RViz.

## Features

- 🚁 **Realistic Quadcopter Dynamics**: Full 6-DOF drone model with accurate physics simulation
- 🎮 **Multiple Control Modes**: 
  - Manual teleoperation via keyboard
  - Autonomous waypoint navigation
  - Position hold and stabilization
- 📡 **Sensor Suite**:
  - IMU (Inertial Measurement Unit)
  - Camera with real-time image streaming
  - GPS positioning (simulated)
- 🎯 **Advanced Control**:
  - Cascaded PID controllers (position, attitude, rate)
  - Motor mixing for X-configuration quadcopter
  - Automatic takeoff and landing sequences
- 🌍 **3D Visualization**: 
  - Real-time visualization in Gazebo Harmonic
  - RViz integration for sensor data and trajectory visualization
- 🔧 **Modular Architecture**: Easy to extend and customize for different drone configurations

## Prerequisites

### System Requirements
- Ubuntu 24.04 LTS (Noble)
- ROS2 Jazzy Jalisco
- Gazebo Harmonic
- At least 8GB RAM recommended
- Graphics card with OpenGL support

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-controller-manager \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  gz-harmonic
```

## Installation

1. **Clone the repository**:
```bash
cd ~
git clone https://github.com/yourusername/drone-perimeter-surveillance.git
cd drone-perimeter-surveillance
```

2. **Build the workspace**:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

3. **Source the workspace**:
```bash
source install/setup.bash
```

## Usage

### Quick Start

Launch the complete simulation with all components:
```bash
ros2 launch quadcopter_simulation gazebo_sim.launch.py
```

### Individual Components

**1. Launch simulation without RViz:**
```bash
ros2 launch quadcopter_simulation gazebo_sim.launch.py use_rviz:=false
```

**2. Run teleoperation only:**
```bash
ros2 run quadcopter_simulation drone_teleop.py
```

**3. Run autonomous waypoint navigation:**
```bash
ros2 run quadcopter_simulation waypoint_follower.py
```

**4. Run the controller node standalone:**
```bash
ros2 run quadcopter_simulation drone_controller
```

### Teleoperation Controls

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
│   │   │   ├── quadcopter.urdf.xacro     # Complete drone model
│   │   │   └── simple_quadcopter.urdf    # Simplified model for testing
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

## Architecture

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
- Use `simple_quadcopter.urdf` for basic testing

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

## Future Enhancements

- [ ] GPS waypoint navigation
- [ ] Obstacle avoidance using lidar
- [ ] Multi-drone swarm coordination
- [ ] Computer vision for object tracking
- [ ] Landing pad detection and precision landing
- [ ] Battery simulation and management
- [ ] Wind disturbance modeling
- [ ] Path planning algorithms (RRT*, A*)

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS2 Community for the excellent documentation
- Gazebo team for the simulation platform
- PX4 project for inspiration on flight dynamics

## Contact

- Project Maintainer: [Your Name]
- Email: developer@example.com
- Project Link: https://github.com/yourusername/drone-perimeter-surveillance

## Citation

If you use this project in your research, please cite:
```bibtex
@software{quadcopter_simulation_2025,
  author = {Your Name},
  title = {ROS2 Quadcopter Drone Simulation},
  year = {2025},
  url = {https://github.com/yourusername/drone-perimeter-surveillance}
}
```
```

This README provides comprehensive documentation for your project, making it professional and easy for others to understand and use. You can customize the contact information, repository links, and add any specific details about your implementation.