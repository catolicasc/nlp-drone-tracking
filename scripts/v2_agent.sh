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

if [ -z "${LVLM_API_KEY:-${OPENAI_API_KEY:-}}" ]; then
  echo "Erro: configure LVLM_API_KEY no .env antes de iniciar o agente V2."
  echo "      A V2 não possui fallback heurístico."
  exit 1
fi

echo "==> Oracle Vision V2: agente VLM-only"
echo "    Task topic: /v2/task"
echo "    Status:     /v2/status"
echo "    Model:      ${LVLM_MODEL:-gpt-4o-mini}"

exec ros2 run v2_vlm_drone v2_vlm_agent --ros-args -p use_sim_time:=false "$@"
