#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FCU_URL="${1:-udp://@127.0.0.1:18570}"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

echo "==> Oracle Vision V2: MAVROS"
echo "    FCU_URL=${FCU_URL}"
exec "$PROJECT_ROOT/scripts/run_mavros_px4.sh" "$FCU_URL"
