#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOPIC_KIND="${1:-status}"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ROS_DISTRO=humble
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi
set -u

case "$TOPIC_KIND" in
  doctor)
    echo "==> Nós ROS"
    ros2 node list || true
    echo
    echo "==> /v2/task"
    ros2 topic info -v /v2/task || true
    echo
    echo "==> /v2/status"
    ros2 topic info -v /v2/status || true
    echo
    echo "==> /mavros/state"
    timeout 5s ros2 topic echo --once /mavros/state || true
    echo
    echo "==> /drone/camera/image_raw"
    ros2 topic info -v /drone/camera/image_raw || true
    ;;
  status)
    exec ros2 topic echo /v2/status
    ;;
  state)
    exec ros2 topic echo /mavros/state
    ;;
  pose)
    exec ros2 topic echo /mavros/local_position/pose
    ;;
  statustext)
    exec ros2 topic echo /mavros/statustext/recv
    ;;
  list)
    exec ros2 topic list
    ;;
  *)
    echo "Uso: $0 [doctor|status|state|pose|statustext|list]"
    exit 2
    ;;
esac
