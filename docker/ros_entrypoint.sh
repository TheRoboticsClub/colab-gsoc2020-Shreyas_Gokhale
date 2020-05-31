#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
sleep 2s
source "/opt/jderobot/share/jderobot/gazebo/assets-setup.sh"
sleep 2s
exec "$@"