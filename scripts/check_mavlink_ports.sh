#!/usr/bin/env bash
# Diagnóstico rápido: portas MAVLink PX4 / QGC / MAVROS.
set -euo pipefail

echo "==> Portas MAVLink (UDP/TCP)"
if command -v ss >/dev/null 2>&1; then
  ss -ulnp 2>/dev/null | grep -E '18570|14550|14580' || echo "    (nenhuma porta UDP MAVLink encontrada)"
  ss -tlnp 2>/dev/null | grep -E ':4560' || echo "    (TCP 4560 HIL não encontrado — sim parada?)"
else
  echo "    'ss' não instalado"
fi

echo
echo "==> Processos"
pgrep -af 'px4_sitl_default/bin/px4' 2>/dev/null || echo "    PX4 SITL: não rodando"
pgrep -af 'QGroundControl' 2>/dev/null || echo "    QGroundControl: não rodando"
pgrep -af 'mavros_node' 2>/dev/null || echo "    MAVROS: não rodando"

echo
echo "==> Checklist QGC"
echo "  1) ./scripts/iniciar_drone.sh  (timeline PLAY)"
echo "  2) QGC: link padrão UDP 14550 (NÃO escutar na 18570)"
echo "  3) Docs: docs/qgroundcontrol.md"
