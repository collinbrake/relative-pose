#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BAG_DIR_DEFAULT="$REPO_ROOT/bags"
BAG_STEM_DEFAULT="run_$(date +%Y%m%d_%H%M%S)"

BAG_DIR="${1:-$BAG_DIR_DEFAULT}"
BAG_STEM="${2:-$BAG_STEM_DEFAULT}"
BAG_PATH="$BAG_DIR/$BAG_STEM"

mkdir -p "$BAG_DIR"

cleanup() {
  local exit_code=$?
  set +e

  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[run_sim_and_record] Stopping bag recorder (pid=$BAG_PID)..."
    kill -INT "$BAG_PID"
    wait "$BAG_PID" 2>/dev/null
  fi

  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[run_sim_and_record] Stopping simulation launch (pid=$LAUNCH_PID)..."
    kill -INT "$LAUNCH_PID"
    wait "$LAUNCH_PID" 2>/dev/null
  fi

  exit "$exit_code"
}
trap cleanup INT TERM EXIT

echo "[run_sim_and_record] Building workspace..."
colcon build --symlink-install

# shellcheck disable=SC1091
set +u
source "$REPO_ROOT/install/setup.bash"
set -u

echo "[run_sim_and_record] Starting simulation..."
ros2 launch relative_pose_sim multi_robot_sim.launch.py &
LAUNCH_PID=$!

# Give Gazebo and robot spawn a short head start for cleaner bag startup.
sleep 5

echo "[run_sim_and_record] Recording bag to: $BAG_PATH"
ros2 bag record -s mcap -o "$BAG_PATH" \
  /clock \
  /tf \
  /tf_static \
  /follower/pointcloud \
  /leader/ground_truth \
  /follower/ground_truth \
  /leader/odom \
  /follower/odom \
  /follower/tf \
  /follower/tf_static \
  /leader/tf \
  /leader/tf_static &
BAG_PID=$!

echo "[run_sim_and_record] Running. Press Ctrl+C to stop launch and recording."

# Keep script alive while either process is alive.
while true; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[run_sim_and_record] Launch process exited; stopping recorder."
    break
  fi
  if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[run_sim_and_record] Bag recorder exited; stopping launch."
    break
  fi
  sleep 1
done

cleanup
