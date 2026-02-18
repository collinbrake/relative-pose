# Simulation

## Specification

- Two robots
  - Rectangle shape: 1m long x 0.75m wide x 1m tall
  - Differential drive
  - Leader robot: pose
  - Follower robot: pose, 3D lidar (mounted to the front side of the follower, back is occluded)
- Environment: flat plane
- Data recording
  - leader pose
  - follower pose
  - lidar pointcloud
  - ROS bag format
- Control:
  - leader robot should follow a scripted path with randomized (but continuous) poses, speeds, and orientations
  - follower robot follows another scripted path that keeps it within lidar range of the leader
    - randomizes their relative distance and angle
    - hard proximity limit: don't let follower get closer than 0.5m to leader
