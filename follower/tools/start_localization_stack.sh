#!/usr/bin/env bash
# 팔로워 위치추정 스택을 띄운다 (모터는 건드리지 않는다).
#
#   ld08_driver  -> /follower/scan_raw
#   lidar_mask   -> /follower/scan          (SLAM/AMCL 은 반드시 이쪽을 쓴다)
#   lidar_tf     -> follower_base_link -> follower_laser_frame
#   serial_bridge-> /follower/imu/data_raw, /follower/motor_command 수신
#   scan_odom    -> /follower/odom_scan
#   ekf_node     -> follower_odom -> follower_base_link (TF 소유자)
#
# 종료: Ctrl-C 하면 자식 전부 정리한다.
set -uo pipefail

set +u
source "$HOME/follower_ws/env.sh"
source "$HOME/lidar_overlay_ws/install/setup.bash"
set -u

root="$HOME/lidar_overlay_ws"
pids=()

cleanup() {
  printf '\n스택 정리 중...\n'
  for pid in "${pids[@]:-}"; do
    [ -n "${pid:-}" ] && kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  exit 0
}
trap cleanup INT TERM

start() {
  local label="$1"; shift
  printf '  [start] %s\n' "$label"
  "$@" &
  pids+=("$!")
}

# ld08.launch.py 는 /scan 으로 내고 frame_id 기본값이 base_scan 이다.
# 사슬(scan_raw -> mask -> scan)과 TF(follower_laser_frame)에 맞추려면 둘 다
# 바꿔야 하는데 그 런치는 remap 을 받지 않는다.  그래서 노드를 직접 띄운다.
start "ld08_driver"   ros2 run ld08_driver ld08_driver --ros-args \
        -p frame_id:=follower_laser_frame \
        -r /scan:=/follower/scan_raw
sleep 3
start "lidar_mask"    python3 "$root/lidar_mask.py"
start "lidar_tf"      ros2 launch "$root/launch/follower_lidar_tf.launch.py"
start "serial_bridge" ros2 run follower_serial_bridge serial_bridge_node \
        --ros-args -r __ns:=/follower -r __node:=serial_bridge \
        --params-file "$root/config/follower_fire.yaml"
sleep 3
start "localization"  ros2 launch "$root/launch/follower_localization.launch.py"

printf '\n스택 기동 완료. Ctrl-C 로 종료.\n'
wait
