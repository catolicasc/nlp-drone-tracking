#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

VALUE=${1:-true}

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: ${VALUE}}"
