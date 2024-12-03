#!/bin/bash
set -e
source /root/.bashrc

ROS_DISTRO=melodic

source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ ! -e ".CONTAINER_INITIALIZED_PLACEHOLDER" ]; then
    echo "-- First container startup --"
    catkin init
    #catkin config
    catkin build
    # This placeholder file used in the github action to check when catkin build is done, do not remove
    touch ".CONTAINER_INITIALIZED_PLACEHOLDER"
    # Source the workspace in every new shell to avoid sourcing it manually
    echo "source /ws/devel/setup.bash" >> /root/.bashrc
else
    echo "-- Not first container startup --"
    source "/ws/devel/setup.bash"
fi

# Execute the container command
exec "$@"