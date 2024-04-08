#!/bin/bash

# This script runs on the host and is used by the desktop application
# First, it starts the debug cameras
# It ensures the botbuilt container is running, then runs the launch_robots
# script inside the container.
# Immediately after exiting the container application, this stops the debug cameras
# After exiting, this opens the nautilus file explorer to view the logs from
# the robot session.
BB_WS="/home/dev/wall_panels/bb_ws"
CONTAINER="botbuilt-amd64-desktop-devel"
DATE=$(date '+%Y%m%d-%H%M%S')

fail () {
  # send message to stderr
  printf "%s\n" "$1" >&2
  # send notification to desktop
  notify-send -t 5000 -i /usr/share/pixmaps/botbuilt_icon.png "Launch Robots" "$1"
  # exit with 2nd argument, or 1 by default
  exit "${2-1}"
}
# ensure the container is running
CONTAINER_RUNNING="$(docker container inspect -f '{{.State.Running}}' "${CONTAINER}")"

if [ "${CONTAINER_RUNNING}" != "true" ]; then
	sudo docker start "${CONTAINER}" || fail "Failed to start container ${CONTAINER}" 1
else
  # ensure the GPU is available inside the container
  GPU_AVAILABLE="$(docker container exec botbuilt-amd64-desktop-devel bash -c 'nvidia-smi; echo $?' | tail -n 1)"
  if [ "${GPU_AVAILABLE}" != "0" ]; then
	  sudo docker restart "${CONTAINER}" || fail "Failed to restart container ${CONTAINER}" 3
  fi
fi

# start the debug cameras
pushd "${HOME}/camera_debug_system" || fail "Cannot find ${HOME}/camera_debug_system." 2
./auto-record-videos.sh start "$DATE" & # || fail "Failed to start debug cameras" 2

# launch the robots
sudo docker exec \
  "${CONTAINER}" \
  "${BB_WS}/src/robot_operation/robot_operation/robot_launcher/launch_robots" || \
  fail "Could not launch robots" 3

if [ -z "$VOLUME_DEV_PATH" ]; then
    VOLUME_DEV_PATH="${HOME}/volume_dev"
fi

# open the file explorer to the correct dir
nautilus "${VOLUME_DEV_PATH}/wall_panels/bb_ws/latest_logs" &

./auto-record-videos.sh stop "$DATE" & # || fail "Failed to stop debug cameras" 2

ROBOT_LOG_FOLDER="${HOME}/robot_logs"
if [ ! -d "${ROBOT_LOG_FOLDER}" ]; then
  mkdir -p "${ROBOT_LOG_FOLDER}"
fi

cp "${VOLUME_DEV_PATH}/wall_panels/bb_ws/latest_logs/latest_logs.tar.xz" \
  "${ROBOT_LOG_FOLDER}/${DATE}_logs.tar.xz"
