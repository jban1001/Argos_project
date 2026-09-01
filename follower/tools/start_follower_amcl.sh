#!/usr/bin/env bash
set -uo pipefail

mode=${1:-}
if [ "$mode" != "--check" ] && [ "$mode" != "--start" ]; then
  printf 'usage: %s --check|--start\n' "$0" >&2
  exit 64
fi

set +u
source "$HOME/follower_ws/env.sh"
source "$HOME/lidar_overlay_ws/install/setup.bash"
set -u

root=$HOME/lidar_overlay_ws
params=$root/config/follower_amcl.yaml
launch_file=$root/launch/follower_amcl.launch.py
stamp=$(date +%Y%m%d_%H%M%S)

abort() {
  printf 'ABORT: %s\n' "$1" >&2
  exit "$2"
}

for package in nav2_amcl nav2_lifecycle_manager tf2_ros; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 \
    || abort "required ROS package is missing: $package" 30
done
[ -f "$params" ] || abort "missing AMCL parameters: $params" 31
[ -f "$launch_file" ] || abort "missing AMCL launch file: $launch_file" 32

if pgrep -af '[c]ooperative_node' >/dev/null \
  || ros2 node list 2>/dev/null | grep -qi cooperative; then
  abort 'cooperative localization is active and would also own map -> follower_odom' 33
fi
if ros2 node list 2>/dev/null | grep -Fxq '/follower/amcl' \
  || pgrep -af '[n]av2_amcl.*/amcl' >/dev/null; then
  abort 'follower AMCL is already running' 34
fi

map_info=$(ros2 topic info -v /map 2>&1)
printf '%s\n' "$map_info" | grep -Eq 'Publisher count: [1-9][0-9]*' \
  || abort '/map has no publisher; start the main robot map publisher first' 40

set +e
map_sample=$(timeout 10s ros2 topic echo /map --once --field header \
  --qos-reliability reliable --qos-durability transient_local \
  --qos-history keep_last --qos-depth 1 2>&1)
map_rc=$?
scan_sample=$(timeout 6s ros2 topic echo /follower/scan --once --field header 2>&1)
scan_rc=$?
odom_tf=$(timeout 6s ros2 run tf2_ros tf2_echo \
  follower_odom follower_base_link 2>&1)
odom_tf_rc=$?
existing_map_tf=$(timeout 4s ros2 run tf2_ros tf2_echo \
  map follower_odom 2>&1)
set -e

[ "$map_rc" -eq 0 ] || abort '/map publisher exists but no OccupancyGrid arrived in 10 seconds' 41
printf '%s\n' "$map_sample" | grep -Eq 'frame_id:[[:space:]]*map([[:space:]]|$)' \
  || abort '/map OccupancyGrid frame_id is not map' 42
[ "$scan_rc" -eq 0 ] || abort '/follower/scan did not deliver a sample' 43
printf '%s\n' "$scan_sample" | grep -Eq 'frame_id:[[:space:]]*follower_laser_frame' \
  || abort '/follower/scan frame_id is not follower_laser_frame' 44
printf '%s\n' "$odom_tf" | grep -q 'Translation:' \
  || abort 'follower_odom -> follower_base_link is unavailable; initialize VIO first' 45
if printf '%s\n' "$existing_map_tf" | grep -q 'Translation:'; then
  abort 'map -> follower_odom already exists; another localization source owns it' 46
fi

printf 'PRECHECK_OK\n'
printf '  map: live /map with frame_id=map\n'
printf '  scan: /follower/scan in follower_laser_frame\n'
printf '  odom: follower_odom -> follower_base_link available\n'
printf '  TF owner: map -> follower_odom is currently free\n'

if [ "$mode" = "--check" ]; then
  exit 0
fi

mkdir -p "$root/log"
log=$root/log/follower_amcl_$stamp.log
nohup setsid ros2 launch "$launch_file" params_file:="$params" \
  >"$log" 2>&1 < /dev/null &
launch_pid=$!
printf 'AMCL_LAUNCH_PID=%s LOG=%s\n' "$launch_pid" "$log"
sleep 8

ros2 node list 2>/dev/null | grep -Fxq '/follower/amcl' \
  || abort "AMCL node did not start; inspect $log" 50
lifecycle=$(ros2 lifecycle get /follower/amcl 2>&1 || true)
printf 'AMCL_LIFECYCLE=%s\n' "$lifecycle"
printf '%s\n' "$lifecycle" | grep -qi active \
  || abort "AMCL did not reach active state; inspect $log" 51

node_info=$(ros2 node info /follower/amcl 2>&1 || true)
printf '%s\n' "$node_info" | grep -Fq '/follower/amcl_pose' \
  || abort 'follower AMCL is not publishing /follower/amcl_pose' 52
printf '%s\n' "$node_info" | grep -Fq '/follower/initialpose' \
  || abort 'follower AMCL is not subscribed to /follower/initialpose' 53

printf 'AMCL_READY: publish a measured initial pose on /follower/initialpose (RViz 2D Pose Estimate).\n'
printf 'Follower pose output: /follower/amcl_pose\n'
printf 'Do not run cooperative_node while AMCL is active.\n'
