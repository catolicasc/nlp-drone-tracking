#!/usr/bin/env bash
# Agente LVLM (camada ARNA) para Oracle Vision.
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

LISTEN_ONLY=0
TASK="${1:-Decole 2 metros e procure pessoas na área.}"
if [ "${1:-}" = "--listen" ]; then
  LISTEN_ONLY=1
  TASK=""
  shift
elif [ "$#" -gt 0 ]; then
  shift
fi

HEURISTIC="${HEURISTIC_MODE:-0}"
EXTRA_ARGS=()
if [ "$HEURISTIC" = "1" ] || [ "$HEURISTIC" = "true" ]; then
  EXTRA_ARGS+=(-p heuristic_mode:=true)
fi

echo "==> Oracle Vision: agente LVLM"
if [ "$LISTEN_ONLY" -eq 1 ]; then
  echo "    Modo: aguardando tarefas em /drone_agent/task"
else
  echo "    Tarefa: $TASK"
fi
echo
echo "Pré-requisitos:"
echo "  A) ./scripts/iniciar_drone.sh"
echo "  B) ./scripts/run_mavros_px4.sh"
echo "  C) ./scripts/run_person_detector.sh"
echo
if [ "$HEURISTIC" != "1" ] && [ "$HEURISTIC" != "true" ]; then
  if [ -z "${LVLM_API_KEY:-}" ] && [ -z "${OPENAI_API_KEY:-}" ]; then
    echo "Aviso: sem LVLM_API_KEY — use HEURISTIC_MODE=1 ou configure .env"
    EXTRA_ARGS+=(-p heuristic_mode:=true)
  fi
fi

if [ "$LISTEN_ONLY" -eq 1 ]; then
  exec ros2 run drone_agent lvlm_agent --ros-args \
    -p use_sim_time:=false \
    "${EXTRA_ARGS[@]}" \
    "$@"
else
  exec ros2 run drone_agent lvlm_agent --ros-args \
    -p "task:=${TASK}" \
    -p use_sim_time:=false \
    "${EXTRA_ARGS[@]}" \
    "$@"
fi
