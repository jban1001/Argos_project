#!/bin/bash
source /home/doorian0615/follower_ws/env.sh >/dev/null 2>&1
source /home/doorian0615/lidar_overlay_ws/install/setup.bash >/dev/null 2>&1
# follow_controller only makes decisions (publish_commands=false); the fire
# supervisor remains the sole actuator owner and gates those decisions by mode.
exec ros2 launch /home/doorian0615/lidar_overlay_ws/launch/follower_fire.launch.py \
  start_follow_controller:=true start_serial_bridge:=false \
  initial_mode:=auto nav2_action:=/follower/navigate_to_pose \
  enable_motion:=true enable_pump:=true
