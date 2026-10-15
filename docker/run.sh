#!/usr/bin/env bash
# Start the driver container with access to the robots (host network), PCAN, cameras and X11.
# The repository is mounted at /home/dev/ros2_ws - the launch files expect that path.
#   bash docker/run.sh            # interactive shell (container name: dual_arm_drivers)
#   bash docker/run.sh <cmd...>   # run a command
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-dual-arm-allegro:jazzy}"
NAME="${NAME:-dual_arm_drivers}"
xhost +local: >/dev/null 2>&1 || true
docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run -it --rm --name "$NAME" \
  --privileged --net=host --ipc=host \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "$WS":/home/dev/ros2_ws \
  "$IMAGE" "${@:-bash}"
