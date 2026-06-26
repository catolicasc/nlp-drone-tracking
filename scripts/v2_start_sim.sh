#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Oracle Vision V2: Isaac Sim + PX4"
exec "$PROJECT_ROOT/scripts/run_isaac.sh" "$@"
