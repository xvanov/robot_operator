#!/bin/bash
husarnet-daemon &
ROS_DISTRO=humble zenoh-bridge-ros2dds --config /operator-dds-config.json5 --connect "tcp/${BOTBUILT_SERVER_NAME}:7447" --domain "${ROS_DOMAIN_ID}" &
exec bash

