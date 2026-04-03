#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

FCU_URL=${1:-"udp://@127.0.0.1:18570"}
GCS_URL=${2:-""}

if [[ -n "${GCS_URL}" ]]; then
  exec ros2 launch mavros px4.launch fcu_url:=${FCU_URL} gcs_url:=${GCS_URL}
else
  exec ros2 launch mavros px4.launch fcu_url:=${FCU_URL}
fi
