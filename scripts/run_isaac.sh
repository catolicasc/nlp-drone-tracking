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

# Evita conflito entre ROS 2 do sistema e o Python do Isaac (padrão Pegasus isaac_run).
unset ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH || true
for ros_path in /opt/ros/humble /opt/ros/jazzy /opt/ros/iron; do
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' - || true)"
  fi
done
export LD_LIBRARY_PATH

if [ -f /etc/os-release ]; then
  UBUNTU_VERSION="$(grep '^VERSION_ID=' /etc/os-release | cut -d'"' -f2)"
else
  UBUNTU_VERSION=""
fi

if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/jazzy/lib"
elif [ -n "$UBUNTU_VERSION" ]; then
  export ROS_DISTRO=humble
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
fi

echo "Rodando Oracle Vision (SimulationApp + Pegasus):"
echo "$SCRIPT_PATH"

exec "$ISAAC_PYTHON" "$SCRIPT_PATH" "$@"
