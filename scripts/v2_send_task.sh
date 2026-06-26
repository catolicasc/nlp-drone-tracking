#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$PROJECT_ROOT/.env" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$PROJECT_ROOT/.env"
  set +a
fi

if [ "$#" -eq 0 ]; then
  echo "Uso: $0 \"decole 1 metro, avance 1 metro e pouse\""
  exit 2
fi

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

TASK="$*"
python3 - "$TASK" <<'PY'
import sys
import time

import rclpy
from std_msgs.msg import String

task = sys.argv[1]
rclpy.init()
node = rclpy.create_node("v2_send_task")
pub = node.create_publisher(String, "/v2/task", 10)
deadline = time.time() + 5.0
while pub.get_subscription_count() == 0 and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if pub.get_subscription_count() == 0:
    print("Erro: nenhum agente V2 está assinando /v2/task.")
    print("Abra outro terminal e rode: ./scripts/v2_agent.sh")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)
msg = String()
msg.data = task
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.2)
print(f"Publicado em /v2/task para {pub.get_subscription_count()} assinante(s): {task}")
node.destroy_node()
rclpy.shutdown()
PY
