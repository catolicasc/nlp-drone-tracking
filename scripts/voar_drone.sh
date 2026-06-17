#!/usr/bin/env bash
# Arma o drone, entra em OFFBOARD e publica setpoint de decolagem.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

Z="${1:-2.0}"
X="${2:-0.0}"
Y="${3:-0.0}"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

echo "==> Oracle Vision: voar drone"
echo "    Setpoint: x=${X} y=${Y} z=${Z}"
echo
echo "Pré-requisitos (2 terminais além deste):"
echo "  A) ./scripts/iniciar_drone.sh   ← sim em PLAY"
echo "  B) ./scripts/run_mavros_px4.sh ← MAVROS em udp://127.0.0.1:18570"
echo

if ! pgrep -f "px4_sitl_default/bin/px4" >/dev/null 2>&1; then
  echo "Aviso: processo PX4 SITL não encontrado. A simulação Isaac está rodando?"
fi

if ! pgrep -f "mavros_node" >/dev/null 2>&1 && ! pgrep -f "mavros.launch" >/dev/null 2>&1; then
  echo "Aviso: MAVROS não parece estar rodando."
  echo "Abra outro terminal e rode: ./scripts/run_mavros_px4.sh"
  echo
fi

exec "$PROJECT_ROOT/scripts/offboard_takeoff.sh" "$Z" "$X" "$Y"
