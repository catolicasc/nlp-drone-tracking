#!/usr/bin/env bash
# Pipeline standalone: MAVROS → setpoints → OFFBOARD → ARM → takeoff.
# Uso: ./scripts/pipeline.sh [z] [x] [y] [rate]
#   z=2.0 x=0.0 y=0.0 rate=20
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

Z=${1:-2.0}
X=${2:-0.0}
Y=${3:-0.0}
RATE=${4:-20}
PRE_SETPOINT_SEC=${PRE_SETPOINT_SEC:-2}
OFFBOARD_RETRIES=${OFFBOARD_RETRIES:-20}
ARM_RETRIES=${ARM_RETRIES:-20}

_ros_setup() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  else
    echo "Erro: ROS 2 não encontrado."
    exit 1
  fi
  if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
  fi
}

set +u
_ros_setup
set -u

echo "==> Pipeline: z=${Z}m x=${X} y=${Y} rate=${RATE}Hz"
echo

# --- 1. Verificar MAVROS ---
echo "[1/6] Verificando MAVROS..."
if ! ros2 topic echo --once /mavros/state --qos-reliability best_effort >/dev/null 2>&1; then
  echo "  ERRO: /mavros/state indisponível. Rode ./scripts/run_mavros_px4.sh"
  exit 1
fi
echo "  OK"

# --- 2. Verificar conexão ---
echo "[2/6] Verificando conexão FCU..."
CONNECTED=0
for i in $(seq 1 30); do
  if ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "connected: (true|True)"; then
    CONNECTED=1
    break
  fi
  sleep 1
done
if [ "$CONNECTED" -eq 0 ]; then
  echo "  ERRO: MAVROS não conectou ao FCU em 30s."
  echo "  Checklist:"
  echo "    1) Isaac Sim rodando e em PLAY?"
  echo "    2) PX4 SITL rodando?"
  echo "    3) ./scripts/run_mavros_px4.sh rodando?"
  exit 1
fi
echo "  OK"

# --- 3. Publicar setpoints em background ---
echo "[3/6] Iniciando setpoint publisher (${RATE}Hz)..."
ros2 topic pub -r "${RATE}" /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}" &
PUB_PID=$!

cleanup() {
  echo
  echo "==> Encerrando pipeline..."
  kill "${PUB_PID}" 2>/dev/null || true
  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'AUTO.LOITER'}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# --- 4. Aquecer setpoints ---
echo "[4/6] Aquecendo setpoints por ${PRE_SETPOINT_SEC}s..."
sleep "${PRE_SETPOINT_SEC}"

# --- 5. Entrar em OFFBOARD ---
echo "[5/6] Entrando em OFFBOARD..."
OFFBOARD_OK=0
for i in $(seq 1 "${OFFBOARD_RETRIES}"); do
  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'OFFBOARD'}" >/dev/null 2>&1 || true
  sleep 0.3
  if ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "mode: '?OFFBOARD'?"; then
    OFFBOARD_OK=1
    echo "  OFFBOARD aceito em ${i} tentativas"
    break
  fi
done
if [ "$OFFBOARD_OK" -eq 0 ]; then
  echo "  ERRO: OFFBOARD não aceito após ${OFFBOARD_RETRIES} tentativas."
  ros2 topic echo --once /mavros/state 2>/dev/null || true
  ros2 topic echo --once /mavros/statustext/recv 2>/dev/null || true
  exit 1
fi

# --- 6. Armar ---
echo "[6/6] Armando..."
sleep 1
ARM_OK=0
for i in $(seq 1 "${ARM_RETRIES}"); do
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    "{value: true}" >/dev/null 2>&1 || true
  sleep 0.5
  if ros2 topic echo --once /mavros/state 2>/dev/null | grep -qE "armed: (true|True)"; then
    ARM_OK=1
    echo "  ARM aceito em ${i} tentativas"
    break
  fi
  PX4_MSG=$(ros2 topic echo --once /mavros/statustext/recv 2>/dev/null | grep "text:" | head -1 || true)
  echo "  tentativa ${i}: ${PX4_MSG}"
done
if [ "$ARM_OK" -eq 0 ]; then
  echo "  ERRO: ARM não aceito após ${ARM_RETRIES} tentativas."
  ros2 topic echo --once /mavros/state 2>/dev/null || true
  ros2 topic echo --once /mavros/statustext/recv 2>/dev/null || true
  exit 1
fi

# --- Manter posição ---
echo
echo "==> Drone ARMED + OFFBOARD. Posição: z=${Z}m"
echo "    Ctrl+C para pousar e sair."
echo
wait "${PUB_PID}"
