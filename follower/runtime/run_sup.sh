#!/bin/bash
source /home/doorian0615/follower_ws/env.sh >/dev/null 2>&1
source /home/doorian0615/lidar_overlay_ws/install/setup.bash >/dev/null 2>&1

# A full bring-up must not make the tracks or pump live merely because the
# script was started. The operator opts in through these environment values.
initial_mode=${ARGOS_INITIAL_MODE:-standby}
enable_motion=${ARGOS_ENABLE_MOTION:-false}
enable_pump=${ARGOS_ENABLE_PUMP:-false}
case "$initial_mode" in auto|follow|coordinate_fire|standby) ;; *)
  echo "ABORT: invalid ARGOS_INITIAL_MODE=$initial_mode" >&2; exit 64;; esac
case "$enable_motion" in true|false) ;; *)
  echo "ABORT: ARGOS_ENABLE_MOTION must be true or false" >&2; exit 64;; esac
case "$enable_pump" in true|false) ;; *)
  echo "ABORT: ARGOS_ENABLE_PUMP must be true or false" >&2; exit 64;; esac
if [ "$enable_pump" = true ] && [ "$enable_motion" != true ]; then
  echo "ABORT: pump cannot be enabled while motion is disabled" >&2
  exit 65
fi
echo "supervisor policy: mode=$initial_mode motion=$enable_motion pump=$enable_pump"

# follow_controller only makes decisions (publish_commands=false); the fire
# supervisor remains the sole actuator owner and gates those decisions by mode.
exec ros2 launch /home/doorian0615/lidar_overlay_ws/launch/follower_fire.launch.py \
  start_follow_controller:=true start_serial_bridge:=false \
  initial_mode:="$initial_mode" nav2_action:=/follower/navigate_to_pose \
  enable_motion:="$enable_motion" enable_pump:="$enable_pump"
