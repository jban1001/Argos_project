#!/bin/bash
# 팔로워 기동 -- 순서가 중요하다.
#
#   1) 기반 스택      라이다 / 마스킹 / serial_bridge / EKF
#   2) 카메라/ArUco  추종 입력만 생성, 모터 명령은 발행하지 않는다
#   3) AMCL          --start 를 빼면 usage 만 찍고 끝난다
#   4) Nav2
#   5) 자세 탐색      -> /follower/initialpose
#   6) Nav2 활성화 확인
#   7) 추종 제어기 + 모드/화재 감독기
#
# 4 를 5 보다 먼저 하는 것이 핵심이다.  planner_server 는 글로벌 코스트맵이
# map -> follower_base_link TF 를 얻어야 활성화되는데, 그 TF 는 AMCL 이
# 초기 자세를 받은 뒤에야 나온다.  순서를 바꾸면
#   "Failed to activate global_costmap because transform ... timed out"
# 으로 planner_server 만 inactive 로 남고, bt_navigator 가 모든 목표를
#   "Action server is inactive. Rejecting the goal."
# 로 거절해서 팔로워가 전혀 움직이지 않는다.
# set -u 는 쓰지 않는다. ROS 의 setup.bash 가 AMENT_TRACE_SETUP_FILES 를
# 미정의 상태로 참조해서 소싱 즉시 종료된다.
source /home/doorian0615/follower_ws/env.sh >/dev/null 2>&1
source /home/doorian0615/lidar_overlay_ws/install/setup.bash >/dev/null 2>&1
# SHM 전송을 끄고 UDP 만 쓴다. /dev/shm 잠금이 쌓이면 로컬 노드가
# 서로 안 보이게 되어 Nav2 활성화가 통째로 막힌다.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/doorian0615/fire_test_logs/fastdds_udp_only.xml
L=/home/doorian0615/fire_test_logs

life_get() {
  timeout 5s ros2 lifecycle get "$1" 2>/dev/null
}

echo "[1/7] 기반 스택"
nohup setsid bash /home/doorian0615/lidar_overlay_ws/tools/start_localization_stack.sh \
  > "$L/g1_loc.log" 2>&1 < /dev/null &
sleep 25

echo "[2/7] 카메라/ArUco (명령 발행 없음)"
NODES=$(ros2 node list 2>/dev/null)
if printf '%s\n' "$NODES" | grep -Fxq '/follower/aruco_pose'; then
  echo "      기존 ArUco 노드를 재사용한다."
elif printf '%s\n' "$NODES" | grep -Fxq '/follower/camera'; then
  echo "      기존 카메라를 재사용하고 ArUco만 기동한다."
  nohup setsid ros2 launch follower_bringup aruco_pose.launch.py \
    > "$L/g2_aruco.log" 2>&1 < /dev/null &
else
  nohup setsid ros2 launch follower_bringup follower.launch.py \
    camera:=true serial_bridge:=false vio:=false aruco:=true \
    cooperative:=false follow:=false publish_commands:=false \
    > "$L/g2_aruco.log" 2>&1 < /dev/null &
fi
for i in $(seq 1 15); do
  ros2 topic info /follower/aruco/status 2>/dev/null \
    | grep -Eq 'Publisher count: [1-9][0-9]*' && break
  sleep 2
done
ros2 topic info /follower/aruco/status 2>/dev/null \
  | grep -Eq 'Publisher count: [1-9][0-9]*' \
  || { echo "      ArUco publisher 기동 실패. 중단."; exit 1; }

echo "[3/7] AMCL"
nohup setsid env FOLLOWER_PEER_HOST=192.168.0.18 \
  bash /home/doorian0615/lidar_overlay_ws/tools/start_follower_amcl.sh --start \
  > "$L/g3_amcl.log" 2>&1 < /dev/null &
for i in $(seq 1 24); do
  life_get /follower/amcl | grep -q "active \[3\]" && break
  sleep 3
done
echo "      amcl = $(life_get /follower/amcl | tail -1)"

echo "[4/7] Nav2"
nohup setsid "$L/run_nav2.sh" > "$L/g4_nav2.log" 2>&1 < /dev/null &
sleep 35

echo "[5/7] 자세 탐색"
python3 /home/doorian0615/lidar_overlay_ws/tools/scan_map_search.py --publish \
  2>&1 | tee "$L/g5_pose.log" | tail -8
sleep 3

echo "[6/7] Nav2 활성화 확인 (자세를 받은 뒤라야 planner 가 올라온다)"
for pass in 1 2 3; do
  for n in controller_server planner_server behavior_server bt_navigator velocity_smoother; do
    life_get "/follower/$n" | grep -q "active \[3\]" \
      || timeout 25 ros2 lifecycle set "/follower/$n" activate >/dev/null 2>&1
  done
  OK=0
  for n in controller_server planner_server behavior_server bt_navigator velocity_smoother; do
    life_get "/follower/$n" | grep -q "active \[3\]" && OK=$((OK+1))
  done
  [ "$OK" -ge 5 ] && break
  sleep 5
done
echo "      Nav2 $OK/5 active"
if [ "$OK" -lt 5 ]; then echo "      활성화 실패. 중단."; exit 1; fi

echo "[7/7] 추종 제어기 + 모드/화재 감독기"
nohup setsid "$L/run_sup.sh" > "$L/g7_sup.log" 2>&1 < /dev/null &
for i in $(seq 1 12); do
  grep -qiE "ready|Traceback" "$L/g7_sup.log" 2>/dev/null && break
  sleep 3
done
grep -iE "ready|Traceback" "$L/g7_sup.log" | tail -1
echo "완료"
