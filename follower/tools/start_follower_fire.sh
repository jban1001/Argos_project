#!/usr/bin/env bash
set -uo pipefail

mode=${1:-}
confirmation=${2:-}
case "$mode" in
  --check|--dry-run|--motion) ;;
  --live)
    if [ "$confirmation" != "I_UNDERSTAND_WATER_WILL_FIRE" ]; then
      printf 'ABORT: --live requires I_UNDERSTAND_WATER_WILL_FIRE\n' >&2
      exit 64
    fi
    ;;
  *)
    printf 'usage: %s --check|--dry-run|--motion|--live I_UNDERSTAND_WATER_WILL_FIRE\n' "$0" >&2
    exit 64
    ;;
esac

set +u
source "$HOME/follower_ws/env.sh"
source "$HOME/lidar_overlay_ws/install/setup.bash"
set -u

root="$HOME/lidar_overlay_ws"
params="$root/config/follower_fire.yaml"
launch_file="$root/launch/follower_fire.launch.py"
stamp=$(date +%Y%m%d_%H%M%S)

abort() {
  printf 'ABORT: %s\n' "$1" >&2
  exit "$2"
}

for package in follower_fire_control follower_serial_bridge tf2_ros; do
  prefix=$(ros2 pkg prefix "$package" 2>/dev/null) \
    || abort "required ROS package is missing: $package" 20
  if [ "$package" != "tf2_ros" ] &&
     [[ "$prefix" != "$root/install/"* ]]; then
    abort "$package resolved to underlay instead of lidar overlay: $prefix" 21
  fi
done
[ -f "$params" ] || abort "missing parameters: $params" 22
[ -f "$launch_file" ] || abort "missing launch: $launch_file" 23

nodes=$(ros2 node list 2>/dev/null)
printf '%s\n' "$nodes" | grep -qi cooperative &&
  abort 'cooperative_node is active and conflicts with follower AMCL TF' 30
printf '%s\n' "$nodes" | grep -Fxq '/follower/fire_supervisor' &&
  abort 'fire supervisor is already running' 31
printf '%s\n' "$nodes" | grep -Fxq '/follower/serial_bridge' ||
  abort 'serial bridge is not running; start follower sensors first' 32
printf '%s\n' "$nodes" | grep -Fxq '/follower/amcl' ||
  abort 'follower AMCL is not running' 33

amcl_state=$(ros2 lifecycle get /follower/amcl 2>&1 || true)
printf '%s\n' "$amcl_state" | grep -qi active ||
  abort "follower AMCL is not active: $amcl_state" 34

map_info=$(ros2 topic info -v /map 2>&1)
printf '%s\n' "$map_info" | grep -Eq 'Publisher count: [1-9][0-9]*' ||
  abort '/map has no publisher; connect/start the main robot map server' 40
main_info=$(ros2 topic info -v /amcl_pose 2>&1)
printf '%s\n' "$main_info" | grep -Eq 'Publisher count: [1-9][0-9]*' ||
  abort '/amcl_pose has no publisher; connect/start main robot localization' 41

set +e
map_sample=$(timeout 8s ros2 topic echo /map --once --field header \
  --qos-reliability reliable --qos-durability transient_local \
  --qos-history keep_last --qos-depth 1 2>&1)
map_rc=$?
main_sample=$(timeout 5s ros2 topic echo /amcl_pose --once --field header 2>&1)
main_rc=$?
scan_sample=$(timeout 5s ros2 topic echo /follower/scan --once --field header 2>&1)
scan_rc=$?
telemetry=$(timeout 5s ros2 topic echo /follower/mcu/telemetry --once --field data 2>&1)
telemetry_rc=$?
global_tf=$(timeout 5s ros2 run tf2_ros tf2_echo map follower_base_link 2>&1)
global_tf_rc=$?
set -e

[ "$map_rc" -eq 0 ] || abort '/map did not deliver OccupancyGrid' 42
printf '%s\n' "$map_sample" | grep -Eq 'frame_id:[[:space:]]*map([[:space:]]|$)' ||
  abort '/map frame_id is not map' 43
[ "$main_rc" -eq 0 ] || abort '/amcl_pose did not deliver a main pose' 44
printf '%s\n' "$main_sample" | grep -Eq 'frame_id:[[:space:]]*map([[:space:]]|$)' ||
  abort '/amcl_pose frame_id is not map' 45
[ "$scan_rc" -eq 0 ] || abort '/follower/scan did not deliver a scan' 46
printf '%s\n' "$scan_sample" | grep -q 'follower_laser_frame' ||
  abort '/follower/scan frame_id is not follower_laser_frame' 47
[ "$telemetry_rc" -eq 0 ] || abort 'MCU telemetry is unavailable' 48
[ "$global_tf_rc" -eq 0 ] || abort 'map -> follower_base_link TF is unavailable' 49
printf '%s\n' "$global_tf" | grep -q 'Translation:' ||
  abort 'map -> follower_base_link TF has no transform' 50

aruco_info=$(ros2 topic info -v /follower/aruco/status 2>&1)
printf '%s\n' "$aruco_info" | grep -Eq 'Publisher count: [1-9][0-9]*' ||
  abort '/follower/aruco/status has no publisher' 51

if [ "$mode" = "--live" ]; then
  printf '%s\n' "$telemetry" | grep -Eq ',P:[01]' ||
    abort 'MCU telemetry lacks P field; flash followingbot_mega_fire v2.1 first' 52
fi

start_follow=true
if printf '%s\n' "$nodes" | grep -Fxq '/follower/follow_controller'; then
  publish_value=$(ros2 param get /follower/follow_controller publish_commands 2>&1)
  printf '%s\n' "$publish_value" | grep -qi false ||
    abort 'existing follow_controller publishes directly; restart it with publish_commands=false' 53
  start_follow=false
fi

printf 'PRECHECK_OK\n'
printf '  main: /map and /amcl_pose live in map frame\n'
printf '  follower: AMCL active, scan + ArUco + MCU telemetry live\n'
printf '  TF: map -> follower_base_link available\n'
printf '  command owner: fire supervisor will arbitrate follow/fire\n'

[ "$mode" = "--check" ] && exit 0

enable_motion=false
enable_pump=false
[ "$mode" = "--motion" ] && enable_motion=true
if [ "$mode" = "--live" ]; then
  enable_motion=true
  enable_pump=true
fi

log="$root/log/follower_fire_${stamp}.log"
nohup setsid ros2 launch "$launch_file" \
  start_follow_controller:="$start_follow" \
  start_serial_bridge:=false \
  enable_motion:="$enable_motion" \
  enable_pump:="$enable_pump" >"$log" 2>&1 < /dev/null &
launch_pid=$!
printf 'FIRE_LAUNCH_PID=%s LOG=%s\n' "$launch_pid" "$log"
sleep 6

ros2 node list 2>/dev/null | grep -Fxq '/follower/fire_supervisor' ||
  abort "fire supervisor did not start; inspect $log" 60

printf 'FIRE_SUPERVISOR_READY mode=%s\n' "$mode"
printf 'Dispatch: /fire/dispatch; status: /follower/fire/status\n'
