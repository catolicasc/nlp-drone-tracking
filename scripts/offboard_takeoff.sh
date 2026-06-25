#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

Z=${1:-2.0}
X=${2:-0.0}
Y=${3:-0.0}
RATE=${4:-20}
VERBOSE=${VERBOSE:-0}
PRE_SETPOINT_SEC=${PRE_SETPOINT_SEC:-2}

_ros_setup() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck source=/opt/ros/humble/setup.bash
    source /opt/ros/humble/setup.bash
  else
    echo "Erro: ROS 2 não encontrado."
    exit 1
  fi
  if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
  fi
}

set +u
_ros_setup
set -u

# Publica setpoints em background (obrigatório antes de OFFBOARD/ARM).
ros2 topic pub -r "${RATE}" /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}" &
PUB_PID=$!

cleanup() {
  kill "${PUB_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_mavros_state() {
  local timeout="${1:-20}"
  local t0
  t0=$(date +%s)
  while true; do
    if ros2 topic echo --once /mavros/state >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - t0 > timeout )); then
      echo "Timeout: /mavros/state indisponível. MAVROS está rodando?"
      exit 1
    fi
    sleep 1
  done
}

wait_mavros_connected() {
  local timeout="${1:-30}"
  local t0
  t0=$(date +%s)
  while true; do
    if ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "connected: (true|True)"; then
      return 0
    fi
    if (( $(date +%s) - t0 > timeout )); then
      echo "Timeout: MAVROS sem connected=true."
      echo "Checklist:"
      echo "  1) ./scripts/iniciar_drone.sh  (sim rodando + timeline PLAY)"
      echo "  2) ./scripts/run_mavros_px4.sh"
      ros2 topic echo --once /mavros/state 2>/dev/null || true
      exit 1
    fi
    sleep 1
  done
}

call_set_mode_offboard() {
  if [[ "${VERBOSE}" == "1" ]]; then
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"
  else
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}" >/dev/null 2>&1 || true
  fi
}

call_arm() {
  if [[ "${VERBOSE}" == "1" ]]; then
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
  else
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" >/dev/null 2>&1 || true
  fi
}

is_offboard() {
  ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "mode: '?OFFBOARD'?"
}

is_armed() {
  ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "armed: (true|True)"
}

print_diagnostics() {
  echo "--- /mavros/state ---"
  ros2 topic echo --once /mavros/state 2>/dev/null || true
  echo "--- /mavros/statustext/recv ---"
  ros2 topic echo --once /mavros/statustext/recv 2>/dev/null || true
  echo "-----------------------"
}

wait_mavros_state 10
wait_mavros_connected 15

echo "Aquecendo setpoints por ${PRE_SETPOINT_SEC}s (PX4 exige stream antes de OFFBOARD)..."
sleep "${PRE_SETPOINT_SEC}"

# Sequência correta PX4: OFFBOARD -> ARM (com setpoints já fluindo).
for _ in $(seq 1 10); do
  call_set_mode_offboard
  if is_offboard; then
    break
  fi
  sleep 0.2
done

for _ in $(seq 1 10); do
  call_arm
  if is_armed; then
    break
  fi
  sleep 0.2
done

if ! is_armed || ! is_offboard; then
  echo "Falha ao armar ou entrar em OFFBOARD."
  print_diagnostics
  echo
  echo "Dicas:"
  echo "  - A simulação Isaac precisa estar em PLAY (não pausada)."
  echo "  - Espere ~10s após iniciar a sim e tente de novo."
  echo "  - Rode com diagnóstico: VERBOSE=1 ./scripts/voar_drone.sh"
  exit 1
fi

echo "OFFBOARD + armed. Mantendo setpoints (Ctrl+C para parar)."
wait "${PUB_PID}"
