#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  source "$PROJECT_ROOT/.env"
  set +a
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

echo "Ambiente ROS 2 carregado."
echo "Workspace: $PROJECT_ROOT/ros2_ws"

exec bash
