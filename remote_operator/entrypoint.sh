#!/bin/bash

# this is a copy of the ros_entrypoint.sh script packaged with 
# the ros ubuntu images. It is included here so our cuda-ros image
# can mimic the behavior of a normal ros image

set -e

# setup ros environment
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash" --

exec "$@"
