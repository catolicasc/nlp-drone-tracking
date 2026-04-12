#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  source "$PROJECT_ROOT/.env"
  set +a
fi

if [ -z "${ISAAC_SIM_PATH:-}" ]; then
  echo "Erro: ISAAC_SIM_PATH não definido no .env"
  exit 1
fi

ISAAC_SIM_APP="${ISAAC_SIM_PATH}/_build/linux-x86_64/release/isaac-sim.sh"
SCRIPT_PATH="$PROJECT_ROOT/apps/isaac_app/standalone/main.py"

if [ ! -f "$ISAAC_SIM_APP" ]; then
  echo "Erro: não encontrei $ISAAC_SIM_APP"
  exit 1
fi

echo "Rodando Isaac Sim com script:"
echo "$SCRIPT_PATH"

"$ISAAC_SIM_APP" --enable isaacsim.ros2.bridge --exec "$SCRIPT_PATH"