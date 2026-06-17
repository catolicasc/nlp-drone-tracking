#!/usr/bin/env bash
# Sobe a simulação Oracle Vision (Isaac Sim + Pegasus + PX4 + câmera ROS2).
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

echo "==> Oracle Vision: iniciando simulação do drone"
echo "    Projeto: $PROJECT_ROOT"
echo

# Evita conflito com SITL antigo preso na porta.
if pgrep -f "px4_sitl_default/bin/px4" >/dev/null 2>&1; then
  echo "==> Encerrando instâncias PX4 SITL antigas..."
  pkill -f "px4_sitl_default/bin/px4" || true
  sleep 1
fi

_cleanup_px4_tmp() {
  local need_sudo=0
  for f in /tmp/px4_lock-0 /tmp/px4-sock-0; do
    [ -e "$f" ] || continue
    if rm -f "$f" 2>/dev/null; then
      echo "==> Removido: $f"
    else
      echo "==> Sem permissão para remover: $f"
      need_sudo=1
    fi
  done
  if [ "$need_sudo" -eq 1 ]; then
    echo
    echo "Arquivos PX4 em /tmp pertencem a outro usuário (geralmente root)."
    echo "Corrija com:"
    echo "  sudo rm -f /tmp/px4_lock-0 /tmp/px4-sock-0"
    echo
    exit 1
  fi
}

_cleanup_px4_tmp

echo "==> Próximo passo (outro terminal), após a sim carregar:"
echo "    QGroundControl: link UDP padrão porta 14550 (ver docs/qgroundcontrol.md)"
echo "    ou MAVROS:      ./scripts/run_mavros_px4.sh"
echo "    ou agente LVLM: ./scripts/run_lvlm_agent.sh \"sua tarefa\""
echo "    ou voar ROS:    ./scripts/voar_drone.sh"
echo

exec "$PROJECT_ROOT/scripts/run_isaac.sh" "$@"
