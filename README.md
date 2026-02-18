# relative-pose

Spec: `sim/sim.md`

## Build

```bash
cd ~/relative-pose
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch relative_pose_sim multi_robot_sim.launch.py
```

## RViz

```bash
rviz2 --ros-args -r /tf:=/follower/tf -r /tf_static:=/follower/tf_static
```

In RViz:
- Set `Fixed Frame` to `base_footprint`.
- Add a `PointCloud2` display.
- Set its topic to `/follower/pointcloud`.
