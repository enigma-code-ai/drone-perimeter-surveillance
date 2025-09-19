# Perimeter Surveillance Quadcopter

Autonomous drone systems for perimeter surveillance.

## Scope Boundaries

This project includes:
- Location and zone patrols
- Thread detection (e.g., intruders, unauthorized vehicles, wildlife encroachments, or environmental hazards like fires or leaks, weapon detection, etc.). 
- Alert system and integration with human security teams.
- Counter-drone features (for rogue drones)
- Automated deterrence: Drones could deploy non-lethal responses like bright lights, sirens, or verbal warnings via speakers.
- Generate automated logs with video evidence for insurance or legal purposes.

## Prerequisites

- ROS 2 Jazzy
- Gazebo Garden (`ros-jazzy-ros-gz-sim`)

## CAD Model

1. Create CAD assembly of the quadcopter in SolidWorks.
2. Export to URDF using the SolidWorks to URDF Exporter plugin (steps: https://www.youtube.com/watch?v=2pT3Zscv97c). This plugin will create a ROS1 package with the URDF and meshes.
3. Convert the ROS1 package to a ROS2 package.

## Build

To build the workspace, navigate to the root of your workspace (`quadcopter_ws`) and run:

```bash
colcon build
```

## Run the Simulation

### Gazebo

To launch the quadcopter in a Gazebo simulation, first source your workspace, then run the launch file:

```bash
source install/setup.bash
ros2 launch quadcopter gazebo.launch.py
```

**Activate Sensors (Required for Gazebo Garden):**
```bash
# Activate IMU sensor
gz service -s /world/empty/entity/system/add --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 1000 --req 'entity {id: 16} plugins {name: "gz::sim::systems::Imu" filename: "gz-sim-imu-system"}'

# Activate Camera sensor  
gz service -s /world/empty/entity/system/add --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 1000 --req 'entity {id: 17} plugins {name: "gz::sim::systems::Sensors" filename: "gz-sim-sensors-system"}'
```

**Available Topics:**
- `/imu` - IMU sensor data
- `/camera` - Camera images
- `/camera_info` - Camera parameters
- `/joint_states` - Propeller positions/velocities

### RViz - TODO

To visualize the robot model in RViz, run the following command:

```bash
source install/setup.bash
ros2 launch quadcopter rviz.launch.py
```
