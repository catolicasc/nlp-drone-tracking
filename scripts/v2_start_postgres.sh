#!/usr/bin/env bash
# Sobe o Postgres do projeto (Docker Compose) em infra/postgres.
# Uso:
#   scripts/v2_start_postgres.sh [up|down|logs|reset]
#
# Comandos:
#   up     - sobe o container em background (padrão)
#   down   - para e remove o container (dados preservados no volume)
#   logs   - segue o log do container
#   reset  - para, remove container E volume (apaga todos os dados!)

set -euo pipefail
cd "$(dirname "$0")/.."

COMPOSE_DIR="infra/postgres"
CMD="${1:-up}"

case "$CMD" in
  up)
    echo "[v2] subindo Postgres em localhost:5433 (admin/meu_banco)..."
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" up -d
    echo "[v2] aguardando healthcheck..."
    for i in {1..30}; do
      if docker compose -f "$COMPOSE_DIR/docker-compose.yml" ps | grep -q "healthy"; then
        echo "[v2] Postgres pronto ✓"
        echo "[v2] connection string: postgresql://admin:senha123@localhost:5433/meu_banco"
        exit 0
      fi
      sleep 1
    done
    echo "[v2] timeout aguardando Postgres — veja logs:" >&2
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" logs
    exit 1
    ;;
  down)
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" down
    ;;
  logs)
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" logs -f
    ;;
  reset)
    echo "[v2] ATENÇÃO: isso apaga TODOS os dados do banco."
    read -rp "Continuar? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || exit 0
    docker compose -f "$COMPOSE_DIR/docker-compose.yml" down -v
    echo "[v2] volume removido."
    ;;
  *)
    echo "Uso: $0 [up|down|logs|reset]" >&2
    exit 2
    ;;
esac
