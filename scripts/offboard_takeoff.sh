#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

Z=${1:-1.5}
X=${2:-0.0}
Y=${3:-0.0}
RATE=${4:-20}

# 1) Start publishing setpoints in background
ros2 topic pub -r ${RATE} /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}" &
PUB_PID=$!

cleanup() {
  kill ${PUB_PID} >/dev/null 2>&1 || true
}
trap cleanup EXIT

# 2) Let PX4 receive some setpoints
sleep 2

# 3) OFFBOARD
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# 4) ARM
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 5) Keep publishing
echo "OFFBOARD + armed. Keeping setpoint publisher running. Ctrl+C to stop."
wait ${PUB_PID}
