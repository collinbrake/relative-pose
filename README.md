# relative-pose

Spec: `sim/sim.md`

## Build (optional)

```bash
cd ~/relative-pose
colcon build --symlink-install
source install/setup.bash
```

`scripts/run_sim_and_record.sh` builds and sources the workspace for you, so manual build is optional.

## Run + Record

```bash
cd ~/relative-pose
./scripts/run_sim_and_record.sh
```

This records into `bags/run_YYYYMMDD_HHMMSS/` by default.

Optional custom output directory and bag name:

```bash
./scripts/run_sim_and_record.sh /tmp/my_bags test_run
```

Press `Ctrl+C` to stop both simulation and bag recording.

## Play Back a Bag

```bash
cd ~/relative-pose
source install/setup.bash
ros2 bag info bags/run_20260224_074957/run_20260224_074957_0.mcap

# play the bag with simulated clock:
ros2 bag play bags/run_20260224_074957/run_20260224_074957_0.mcap --clock
```

## Visualize in RViz During Playback

In a new terminal:

```bash
cd ~/relative-pose
source install/setup.bash
rviz2 --ros-args -p use_sim_time:=true -r /tf:=/follower/tf -r /tf_static:=/follower/tf_static
```

In RViz:
- Set `Fixed Frame` to `base_footprint`.
- Add a `PointCloud2` display.
- Set its topic to `/follower/pointcloud`.
