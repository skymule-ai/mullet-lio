#!/usr/bin/env bash
set -e

source /usr/local/share/mullet-ros/setup.bash  # ROS 2 Jazzy + the base workspace (px4_msgs)
source /opt/livox_ws/install/setup.bash        # livox_ros_driver2 (CustomMsg)
source /opt/fast_lio_ws/install/setup.bash

exec "$@"
