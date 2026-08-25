#!/usr/bin/env bash
set -eo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set; run this from a graphical desktop session."
  exit 1
fi

# xorgxrdp provides a virtual Xorg display. Mesa software rendering is more
# reliable there than attempting to attach the Jetson GPU to the virtual Xorg.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

source /opt/ros/jazzy/setup.bash
source /home/odyssey/ydlidar_ws/install/setup.bash
source /home/odyssey/argos_project/ros2_ws/install/setup.bash

exec ros2 launch argos_bringup argos_navigation_mppi.launch.py use_rviz:=true
