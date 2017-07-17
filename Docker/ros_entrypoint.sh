#!/bin/bash
set -e

# setup ros environment
#source "/opt/ros/$ROS_DISTRO/setup.bash"
echo 'source "/opt/ros/indigo/setup.bash"' >> ~/.bashrc
echo 'source "/ROS_Navigation/catkin_ws/devel/setup.bash"' >> ~/.bashrc
exec "$@"
