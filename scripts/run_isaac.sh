#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

if [ -z "${ISAAC_SIM_PATH:-}" ]; then
  echo "Erro: ISAAC_SIM_PATH não definido no .env"
  exit 1
fi

ISAAC_PYTHON="${ISAAC_SIM_PATH}/_build/linux-x86_64/release/python.sh"
SCRIPT_PATH="$PROJECT_ROOT/apps/isaac_app/standalone/main.py"

if [ ! -x "$ISAAC_PYTHON" ]; then
  echo "Erro: não encontrei $ISAAC_PYTHON"
  exit 1
fi

if [ -f /etc/os-release ]; then
  UBUNTU_VERSION="$(grep '^VERSION_ID=' /etc/os-release | cut -d'"' -f2)"
else
  UBUNTU_VERSION=""
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    export ROS_DISTRO=jazzy
  else
    export ROS_DISTRO=humble
  fi
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

ISAAC_RUNTIME_DIR="$(dirname "$ISAAC_PYTHON")"
BRIDGE_LIB_DIR="$ISAAC_RUNTIME_DIR/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

if [ -d "$BRIDGE_LIB_DIR" ]; then
  # Isaac Sim package com bibliotecas ROS 2 leves embutidas.
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${BRIDGE_LIB_DIR}"
  echo "ROS2 Bridge: usando libs embutidas do Isaac ($BRIDGE_LIB_DIR)"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # Isaac Sim 5.1/source tree pode não trazer as libs embutidas; use ROS do sistema.
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  echo "ROS2 Bridge: usando ROS do sistema (/opt/ros/${ROS_DISTRO})"
else
  echo "Erro: não encontrei libs do ROS2 Bridge nem /opt/ros/${ROS_DISTRO}/setup.bash"
  echo "Instale ROS 2 ${ROS_DISTRO} ou verifique ISAAC_SIM_PATH."
  exit 1
fi

echo "Rodando Oracle Vision (SimulationApp + Pegasus):"
echo "$SCRIPT_PATH"
echo "ROS_DISTRO=${ROS_DISTRO} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

exec "$ISAAC_PYTHON" "$SCRIPT_PATH" "$@"
