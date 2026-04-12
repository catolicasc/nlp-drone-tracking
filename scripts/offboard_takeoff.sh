#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

Z=${1:-1.5}
X=${2:-0.0}
Y=${3:-0.0}
RATE=${4:-20}
VERBOSE=${VERBOSE:-0}

# 1) Start publishing setpoints in background
ros2 topic pub -r ${RATE} /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}" &
PUB_PID=$!

cleanup() {
  kill ${PUB_PID} >/dev/null 2>&1 || true
}
trap cleanup EXIT

STATE_TIMEOUT_SEC=${STATE_TIMEOUT_SEC:-15}
T0=$(date +%s)
while true; do
  if ros2 topic echo --once /mavros/state >/dev/null 2>&1; then
    break
  fi
  NOW=$(date +%s)
  if (( NOW - T0 > STATE_TIMEOUT_SEC )); then
    echo "Timeout esperando /mavros/state. MAVROS está rodando?"
    exit 1
  fi
  sleep 1
done

sleep 3

call_set_mode_offboard() {
  if [[ "${VERBOSE}" == "1" ]]; then
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}" || true
  else
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}" >/dev/null 2>&1 || true
  fi
}

call_arm() {
  if [[ "${VERBOSE}" == "1" ]]; then
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" || true
  else
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" >/dev/null 2>&1 || true
  fi
}

is_offboard() {
  ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "mode: OFFBOARD" || \
    ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "mode: 'OFFBOARD'"
}

is_armed() {
  ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "armed: True" || \
    ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "armed: true"
}

print_last_statustext() {
  echo "--- /mavros/statustext/recv (última msg) ---"
  ros2 topic echo --once /mavros/statustext/recv 2>/dev/null || true
  echo "-----------------------------------------"
}

# Tentativa 1: arm -> offboard
for _ in $(seq 1 10); do
  call_arm
  if is_armed; then
    break
  fi
  sleep 0.3
done

for _ in $(seq 1 10); do
  call_set_mode_offboard
  if is_offboard; then
    break
  fi
  sleep 0.3
done

# Tentativa 2 (fallback): offboard -> arm
if ! is_armed; then
  for _ in $(seq 1 10); do
    call_set_mode_offboard
    if is_offboard; then
      break
    fi
    sleep 0.3
  done

  for _ in $(seq 1 10); do
    call_arm
    if is_armed; then
      break
    fi
    sleep 0.3
  done
fi

if ! is_armed; then
  echo "Falha ao armar. Estado atual:"
  ros2 topic echo --once /mavros/state 2>/dev/null || true
  print_last_statustext
  echo "Dica: normalmente é preflight fail (ex.: vertical velocity unstable). Espere alguns segundos e tente de novo."
  exit 1
fi

# 5) Keep publishing
echo "OFFBOARD + armed. Keeping setpoint publisher running. Ctrl+C to stop."
wait ${PUB_PID}
