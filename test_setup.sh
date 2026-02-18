#!/bin/bash

# Quick test script for relative-pose simulation

echo "=========================================="
echo "Relative Pose Simulation - Quick Test"
echo "=========================================="
echo ""

# Check if package is built and sourced
echo "Checking package installation..."
if ros2 pkg list | grep -q relative_pose_sim; then
    echo "✓ Package 'relative_pose_sim' found"
else
    echo "✗ Package 'relative_pose_sim' not found"
    echo "  Please build and source the package:"
    echo "  cd ~/relative-pose"
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
    exit 1
fi

# Check if Gazebo is available
echo "Checking Gazebo installation..."
if command -v gazebo &> /dev/null; then
    echo "✓ Gazebo found"
else
    echo "✗ Gazebo not found"
    echo "  Please install Gazebo for ROS 2"
    exit 1
fi

# Check if models directory exists
echo "Checking model files..."
PACKAGE_PATH=$(ros2 pkg prefix relative_pose_sim)
if [ -f "$PACKAGE_PATH/share/relative_pose_sim/models/leader_robot.sdf" ]; then
    echo "✓ Leader robot model found"
else
    echo "✗ Leader robot model not found"
    exit 1
fi

if [ -f "$PACKAGE_PATH/share/relative_pose_sim/models/follower_robot.sdf" ]; then
    echo "✓ Follower robot model found"
else
    echo "✗ Follower robot model not found"
    exit 1
fi

# Check if world file exists
echo "Checking world file..."
if [ -f "$PACKAGE_PATH/share/relative_pose_sim/worlds/flat_plane.world" ]; then
    echo "✓ World file found"
else
    echo "✗ World file not found"
    exit 1
fi

# Check if controller scripts exist
echo "Checking controller scripts..."
if [ -x "$PACKAGE_PATH/lib/relative_pose_sim/leader_controller.py" ]; then
    echo "✓ Leader controller found and executable"
else
    echo "✗ Leader controller not found or not executable"
    exit 1
fi

if [ -x "$PACKAGE_PATH/lib/relative_pose_sim/follower_controller.py" ]; then
    echo "✓ Follower controller found and executable"
else
    echo "✗ Follower controller not found or not executable"
    exit 1
fi

if [ -x "$PACKAGE_PATH/lib/relative_pose_sim/data_recorder.py" ]; then
    echo "✓ Data recorder found and executable"
else
    echo "✗ Data recorder not found or not executable"
    exit 1
fi

echo ""
echo "=========================================="
echo "All checks passed! ✓"
echo "=========================================="
echo ""
echo "To launch the simulation, run:"
echo "  ros2 launch relative_pose_sim multi_robot_sim.launch.py"
echo ""
echo "To visualize in RViz (follower with LIDAR):"
echo "  rviz2 --ros-args -r /tf:=/follower/tf -r /tf_static:=/follower/tf_static"
echo ""
