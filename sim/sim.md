# Simulation

## Specification

- Two robots
  - Rectangle shape: 1m long x 0.75m wide x 1m tall
  - Differential drive
  - Leader robot: pose
  - Follower robot: pose, 3D lidar
- Environment: flat plane
- Data recording
  - leader pose
  - follower pose
  - lidar pointcloud
- Control: leader robot should follow a scripted path with randomized (but continuous) poses, speeds, and orientations while the follower robot follows another scripted path that keeps it within lidar range of the leader but randomizes their relative distance and angle

## Implementation

### Completed Components

✅ **Package Structure** (`relative_pose_sim`)
- CMakeLists.txt with proper dependencies
- package.xml for ROS2 package management
- Python package structure for controllers

✅ **Robot Models**
- **Leader Robot** (`models/leader_robot.sdf`)
  - 2.5m × 1.5m × 1.5m rectangular body (blue)
  - Differential drive with 1.6m wheel separation
  - Ground truth pose plugin (50Hz)
  - Odometry publishing
- **Follower Robot** (`models/follower_robot.sdf`)
  - 2.5m × 1.5m × 1.5m rectangular body (red)
  - Differential drive with 1.6m wheel separation
  - 3D LIDAR sensor (32×1024, 30m range, 10Hz)
  - Ground truth pose plugin (50Hz)

✅ **Gazebo World** (`worlds/flat_plane.world`)
- Flat ground plane (100m × 100m)
- Optimized physics: 20ms time step, 50Hz update rate
- Configured for real-time factor of 1

✅ **Launch System** (`sim/launch/multi_robot_sim.launch.py`)
- Sequential robot spawning (prevents Gazebo overload)
- Proper namespace isolation (`/leader`, `/follower`)
- TF topic remapping for multi-robot support
- Delayed controller start (5s after spawn)
- Automatic data recording (starts at 6s)

✅ **Leader Controller** (`scripts/leader_controller.py`)
- Figure-8 (lemniscate) path following
- Base speed: 0.3-0.8 m/s (randomized)
- Continuous speed variation using sinusoidal noise
- Continuous angular velocity variation
- Automatic parameter regeneration every ~10s

✅ **Follower Controller** (`scripts/follower_controller.py`)
- Maintains position relative to leader
- Desired distance: 5-15m (within LIDAR range)
- Desired angle: randomized around leader
- Continuous position variation using sinusoidal noise
- Proportional control with tunable gains
- Automatic parameter regeneration every ~15s

✅ **Data Recorder** (`scripts/data_recorder.py`)
- Records to `~/relative_pose_data_<timestamp>/`
- Leader pose CSV: timestamp, position, orientation, velocities
- Follower pose CSV: timestamp, position, orientation, velocities
- LIDAR info CSV: metadata for each point cloud
- Point clouds: binary files (XYZ, float32)
- Recording rate: 50Hz for poses, 10Hz for LIDAR

### Usage

```bash
# Build and source
cd ~/relative-pose
colcon build --symlink-install
source install/setup.bash

# Test installation
./test_setup.sh

# Launch simulation
ros2 launch relative_pose_sim multi_robot_sim.launch.py

# Visualize follower LIDAR in RViz
rviz2 --ros-args -r /tf:=/follower/tf -r /tf_static:=/follower/tf_static
```

### Technical Details

**Namespace Management**
- Each robot operates in isolated namespace
- TF topics remapped: `/tf` → `/<robot>/tf`
- Prevents topic/frame conflicts
- Based on [multi-robot Gazebo article](https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620)

**Control Strategy**
- Leader: Parametric path following with noise injection
- Follower: Relative positioning with proportional control
- Both use continuous randomization (not discrete jumps)

**Data Format**
- Poses: CSV with full state (position, quaternion, euler, velocities)
- Point clouds: Binary numpy arrays (efficient, numpy-compatible)
- Synchronized timestamps for sensor fusion applications

### Directory Structure

```
relative-pose/
├── CMakeLists.txt
├── package.xml
├── setup.cfg
├── README.md
├── test_setup.sh
├── models/
│   ├── box_robot.urdf.xacro
│   ├── leader_robot.sdf
│   └── follower_robot.sdf
├── worlds/
│   └── flat_plane.world
├── scripts/
│   ├── leader_controller.py
│   ├── follower_controller.py
│   └── data_recorder.py
├── sim/
│   ├── sim.md
│   └── launch/
│       ├── multi_robot_sim.launch.py
│       └── turtlesim_mimic.py
└── relative_pose_sim/
    └── __init__.py
```