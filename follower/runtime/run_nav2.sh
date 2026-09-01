#!/bin/bash
source /home/doorian0615/follower_ws/env.sh >/dev/null 2>&1
source /home/doorian0615/lidar_overlay_ws/install/setup.bash >/dev/null 2>&1
exec ros2 launch /home/doorian0615/lidar_overlay_ws/launch/follower_nav2.launch.py
