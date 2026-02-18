# Relative Pose Simulation

Multi-robot simulation for leader-follower scenarios with 3D LIDAR data collection.

## Overview

This package implements a Gazebo simulation with two custom rectangular robots (2.5m x 1.5m x 1.5m):
- **Leader Robot**: Blue robot with differential drive and pose sensing
- **Follower Robot**: Red robot with differential drive and 3D LIDAR sensor

The simulation features:
- Leader follows a randomized figure-8 path with continuous variation in speed and orientation
- Follower maintains position within LIDAR range with randomized relative distance/angle
- Automatic data recording of poses and point clouds

## Building

```bash
cd ~/relative-pose
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Launch the full simulation:

```bash
ros2 launch relative_pose_sim multi_robot_sim.launch.py
```

This will:
1. Start Gazebo with a flat plane world
2. Spawn the leader robot (blue) at origin
3. Spawn the follower robot (red) 5m behind
4. Start both robot controllers
5. Start data recording

### Visualization in RViz

Since the robots use namespaced TF topics, you need to remap them for RViz:

**For Leader Robot:**
```bash
rviz2 --ros-args -r /tf:=/leader/tf -r /tf_static:=/leader/tf_static
```

Then in RViz:
- Set Fixed Frame to `base_footprint` or `odom`
- Add `Odometry` display and set topic to `/leader/ground_truth`

**For Follower Robot (with LIDAR):**
```bash
rviz2 --ros-args -r /tf:=/follower/tf -r /tf_static:=/follower/tf_static
```

Then in RViz:
- Set Fixed Frame to `base_footprint` or `odom`
- Add `PointCloud2` display and set topic to `/follower/pointcloud`
- Add `Odometry` display and set topic to `/follower/ground_truth`

## Topics

### Leader Robot
- `/leader/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/leader/odom` - Odometry from differential drive (nav_msgs/Odometry)
- `/leader/ground_truth` - Ground truth pose (nav_msgs/Odometry)
- `/leader/tf` - Transform tree
- `/leader/tf_static` - Static transforms

### Follower Robot
- `/follower/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/follower/odom` - Odometry from differential drive (nav_msgs/Odometry)
- `/follower/ground_truth` - Ground truth pose (nav_msgs/Odometry)
- `/follower/pointcloud` - 3D LIDAR point cloud (sensor_msgs/PointCloud2)
- `/follower/tf` - Transform tree
- `/follower/tf_static` - Static transforms

## Data Recording

Data is automatically recorded to `~/relative_pose_data_<timestamp>/`:

- `leader_pose.csv` - Leader robot poses (timestamp, position, orientation, velocities)
- `follower_pose.csv` - Follower robot poses (timestamp, position, orientation, velocities)
- `lidar_info.csv` - Metadata about point clouds (timestamp, frame, dimensions, filename)
- `pointclouds/` - Binary files containing XYZ point cloud data

### Loading Point Cloud Data

```python
import numpy as np

# Load a point cloud
points = np.fromfile('pointcloud_000001.bin', dtype=np.float32).reshape(-1, 3)
print(f"Point cloud shape: {points.shape}")  # (N, 3) where N is number of points
```

## Implementation Details

Based on the article: [Efficient Deployment and Operation of Multiple TurtleBot3 Robots in Gazebo](https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620)

Key features:
- **Namespacing**: Each robot has its own namespace (`/leader`, `/follower`)
- **TF Remapping**: TF topics are remapped to avoid conflicts (`/tf` → `/leader/tf`, `/follower/tf`)
- **Sequential Spawning**: Robots spawn sequentially to avoid overwhelming Gazebo
- **Delayed Controller Start**: Controllers start after robots are fully spawned
- **Physics Configuration**: Optimized for real-time simulation with 20ms time steps

## Customization

### Modify Robot Behavior

Edit controller parameters in:
- `scripts/leader_controller.py` - Change path pattern, speed ranges
- `scripts/follower_controller.py` - Change following distance, relative angle

### Modify Robot Models

Edit SDF files:
- `models/leader_robot.sdf` - Leader robot configuration
- `models/follower_robot.sdf` - Follower robot and LIDAR configuration

### Modify World

Edit `worlds/flat_plane.world` to add obstacles, change lighting, etc.

## Troubleshooting

### No points showing in RViz
- Ensure TF remapping matches robot namespace
- Check that Fixed Frame exists in the robot's TF tree
- Verify topic names match the namespace

### Robots not moving
- Check controller nodes are running: `ros2 node list`
- Check velocity commands: `ros2 topic echo /leader/cmd_vel`

### Data not recording
- Check data_recorder node is running: `ros2 node list | grep data_recorder`
- Check output directory exists: `ls ~/relative_pose_data_*`

## License

MIT License - See LICENSE file for details
