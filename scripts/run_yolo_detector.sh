#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

# shellcheck source=lib/yolo_env.sh
source "$PROJECT_ROOT/scripts/lib/yolo_env.sh"

echo "==> Oracle Vision V2: YOLO person detector"
echo "    Camera:     /drone/camera/image_raw"
echo "    Detections: /v2/yolo/detections"
echo "    Model:      ${YOLO_MODEL:-yolov8n.pt}"

exec ros2 run v2_vlm_drone v2_yolo_person_detector --ros-args -p use_sim_time:=false "$@"
