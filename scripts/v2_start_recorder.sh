#!/usr/bin/env bash
# Sobe o nó v2_experiment_recorder, que grava eventos /v2/status no Postgres.
# Precisa: Postgres rodando (scripts/v2_start_postgres.sh) e workspace buildado.
#
# Uso:
#   scripts/v2_start_recorder.sh [experiment_id]
# Exemplo:
#   scripts/v2_start_recorder.sh cenario_A_run1

set -euo pipefail
cd "$(dirname "$0")/.."

# carrega .env se existir (V2_DB_URL vem de lá)
set -a
[ -f .env ] && source .env
set +a

EXP_ID="${1:-}"
DB_URL="${V2_DB_URL:-postgresql://admin:senha123@localhost:5433/meu_banco}"

source install/setup.bash

ROS_ARGS=(--ros-args -p "db_url:=${DB_URL}")
if [ -n "$EXP_ID" ]; then
  ROS_ARGS+=(-p "experiment_id:=${EXP_ID}")
fi

echo "[v2] recorder subindo | db=${DB_URL} | experiment_id=${EXP_ID:-(auto)}"
exec ros2 run v2_vlm_drone v2_experiment_recorder "${ROS_ARGS[@]}"
