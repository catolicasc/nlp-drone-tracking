#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI_DIR="$PROJECT_ROOT/gui"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ROS_DISTRO=humble
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi
set -u

# Prefere venv dedicado; se python3-venv não estiver disponível (Ubuntu),
# instala as dependências com pip --user (PEP 668 exige --break-system-packages).
if python3 -c "import PySide6" 2>/dev/null; then
  echo "==> Dependências da GUI já disponíveis no Python do sistema"
else
  echo "==> Instalando dependências da GUI (pip --user)"
  python3 -m pip install --user --break-system-packages -r "$GUI_DIR/requirements.txt"
fi

echo "==> Oracle Vision V2: GUI de controle"
exec python3 "$GUI_DIR/main.py" "$@"
