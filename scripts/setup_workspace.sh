#!/usr/bin/env bash
# Fetch the pinned upstream drivers, apply the local RealSense patch and build.
# Run inside the container (bash docker/run.sh), from /home/dev/ros2_ws.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [ ! -d src/Universal_Robots_ROS2_Driver ] || [ ! -d src/realsense-ros ]; then
  vcs import src < dual_arm.repos
fi
if git -C src/realsense-ros apply --check ../../patches/realsense-ros.patch 2>/dev/null; then
  git -C src/realsense-ros apply ../../patches/realsense-ros.patch
  echo "applied patches/realsense-ros.patch"
else
  echo "realsense-ros patch already applied (or tree modified) - skipping"
fi
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
echo "done - now: source install/setup.bash"
