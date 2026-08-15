#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

# MAVROS exige o geoid do GeographicLib (egm96-5). Sem ele o nó morre com
# "UAS: GeographicLib exception: File not readable egm96-5.pgm".
# Preferência: instalação do sistema (/usr/share/GeographicLib); senão a
# instalação de usuário feita com:
#   geographiclib-get-geoids -p ~/.local/share/GeographicLib egm96-5
if [ -z "${GEOGRAPHICLIB_DATA:-}" ]; then
  if [ -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]; then
    export GEOGRAPHICLIB_DATA=/usr/share/GeographicLib
  elif [ -f "$HOME/.local/share/GeographicLib/geoids/egm96-5.pgm" ]; then
    export GEOGRAPHICLIB_DATA="$HOME/.local/share/GeographicLib"
  else
    echo "ERRO: dataset geoid egm96-5 do GeographicLib não encontrado." >&2
    echo "  Rode: geographiclib-get-geoids -p ~/.local/share/GeographicLib egm96-5" >&2
    echo "  (ou com sudo: geographiclib-get-geoids egm96-5)" >&2
    exit 1
  fi
fi

FCU_URL=${1:-"udp://@127.0.0.1:18570"}
GCS_URL=${2:-""}

if [[ -n "${GCS_URL}" ]]; then
  exec ros2 launch mavros px4.launch fcu_url:=${FCU_URL} gcs_url:=${GCS_URL} use_sim_time:=false
else
  exec ros2 launch mavros px4.launch fcu_url:=${FCU_URL} use_sim_time:=false
fi
