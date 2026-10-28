#!/usr/bin/env bash
# POC isolada: pessoas animadas via IRA (sem drone/PX4/MAVROS/ROS).
# Uso: ./scripts/poc_ira_people.sh   (HEADLESS=1 p/ sem janela; POC_SECONDS=60; POC_PEOPLE=10)
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

if [ -x "${ISAAC_SIM_PATH}/python.sh" ]; then
  ISAAC_PYTHON="${ISAAC_SIM_PATH}/python.sh"
else
  echo "Erro: não encontrei python.sh do Isaac Sim em $ISAAC_SIM_PATH"
  exit 1
fi

exec "$ISAAC_PYTHON" "$PROJECT_ROOT/apps/isaac_app/standalone/poc_ira_people.py" "$@"
