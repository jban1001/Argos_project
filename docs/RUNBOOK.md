# ARGOS 기동 절차서

2026-09-02 작성. 메인봇(`odyssey`, 192.168.0.18, Jetson Orin) +
팔로워봇(`doorian0615`, 192.168.0.107, Raspberry Pi).

이 문서는 사람과 에이전트가 같이 읽는다. **순서를 지켜야 한다.** 아래
의존 관계는 전부 실제로 실패해 보고 알아낸 것이다.

---

## 0. 절대 순서

```
1) 메인 Nav2 스택        <- map_server 가 /map 을 낸다
2) 팔로워 전체           <- 팔로워 AMCL 은 /map 없으면 ABORT 한다
3) 메인 인지 / 자세 / 순찰  <- 마지막. 이때부터 로봇이 움직인다
```

**팔로워를 먼저 띄울 수 없다.** `start_follower_amcl.sh` 는 프리체크에서
`ABORT: /map has no publisher; start the main robot map publisher first`
로 죽는다. `/map` 은 메인봇 map_server 가 낸다.

각 로봇 안에서도 순서가 있다.

| 로봇 | 반드시 먼저 | 그 다음 | 이유 |
|---|---|---|---|
| 메인 | 자세 탐색 | 순찰 노드 | 위치추정 전에 불을 보면 어긋난 좌표가 팔로워로 간다 |
| 팔로워 | 자세 탐색 | Nav2 활성화 확인 | `planner_server` 는 `map -> follower_base_link` TF 가 있어야 활성화된다. 그 TF 는 AMCL 이 초기 자세를 받아야 나온다 |

메인 순찰 노드는 뜬 뒤에 `/initialpose` 를 **한 번 더** 받아야 시작한다.
자세 탐색이 먼저 발행한 것은 노드가 없을 때라 놓친다. 기동 스크립트가
찾은 좌표를 기억했다가 재발행한다.

---

## 1. 기동

### 메인봇 (1단계: 스택만)

```bash
~/fire_test_logs/run_navstack.sh
```

`Managed nodes are active` 가 두 번 나오면 된다. 안 나오면 라이프사이클
매니저가 4 s 타임아웃으로 포기한 것이니 수동으로 올린다.

```bash
source ~/argos_project/scripts/argos_env.sh
for n in map_server amcl controller_server planner_server bt_navigator \
         velocity_smoother behavior_server smoother_server waypoint_follower; do
  ros2 lifecycle get /$n | grep -q "active \[3\]" || ros2 lifecycle set /$n activate
done
```

`/map` 이 나오는지 확인한 뒤 팔로워로 넘어간다.

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic info /map
```

### 팔로워봇 (전체)

```bash
ssh doorian0615@192.168.0.107 -t '~/fire_test_logs/run_follower_bringup.sh'
```

기반 스택 -> AMCL -> Nav2 -> 자세 탐색 -> Nav2 활성화 확인 5/5 -> 감독기.
5/5 가 안 되면 스크립트가 중단한다. 그 상태로 진행하면 `bt_navigator` 가
모든 목표를 `Action server is inactive. Rejecting the goal.` 로 거절해서
팔로워가 전혀 움직이지 않는다.

### 메인봇 (2단계: 인지 + 자세 + 순찰)

```bash
~/fire_test_logs/run_main_bringup.sh
```

주의: 이 스크립트는 1단계의 Nav2 스택도 같이 띄운다. 위에서 이미 띄웠다면
스택 부분은 건너뛰고 인지부터 손으로 해도 된다.

```bash
~/fire_test_logs/run_perception.sh                                   # 인지
source ~/argos_project/scripts/argos_env.sh \
  && python3 ~/fire_test_logs/scan_map_search_main.py --publish       # 자세
~/fire_test_logs/run_patrol.sh                                        # 순찰
# 순찰 노드가 뜬 뒤 위에서 찾은 좌표를 /initialpose 로 재발행
```

---

## 2. 인지만 단독으로 (MLP + 텔레그램)

Nav2, 순찰, 팔로워 없이 이것만 돌려도 된다.

```bash
~/fire_test_logs/run_perception_only.sh
```

경보 기준은 `fire_prob_threshold` 로 준다 (현재 0.80). `0` 이면 원본
값(0.70)을 쓴다. 지도 위 로봇 위치 표시는 map_server 가 없으면
"가져오지 못했습니다" 로 나온다.

---

## 3. 종료

**launch 를 Ctrl-C 해도 자식 노드가 살아남는다.** LiDAR 와 모터 시리얼을
계속 잡고 있어서 다음 기동이 실패한다.

메인봇:

```bash
pkill -f fire_nav_integrated; pkill -f fire_perception_main; pkill -f argos_navigation.launch; sleep 3; pkill -f "/opt/ros/jazzy/lib/"; pkill -f ydlidar_ros2_driver_node; pkill -f argos_base_driver; pkill -f mpu6050_node; pkill -f wheel_odometry_node; pkill -f scan_normalizer
```

팔로워:

```bash
ssh doorian0615@192.168.0.107 'pkill -f fire_supervisor; sleep 2; pkill -f "/opt/ros/jazzy/lib/"; pkill -f ld08_driver; pkill -f lidar_mask; pkill -f serial_bridge; pkill -f cmd_vel_bridge; pkill -f scan_odom'
```

**종료 후 공유메모리를 정리한다** (4 번 항목 참고).

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ssh doorian0615@192.168.0.107 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*'
```

펌프는 감독기가 죽으면 펌웨어 deadman 이 자동으로 끈다. 릴레이가 열린 채
남지 않는다.

---

## 4. 함정 모음

### 4-1. Fast DDS 공유메모리가 쌓이면 로컬 노드가 서로 안 보인다

`kill -9` 를 반복하면 `/dev/shm` 에 `fastrtps_*` 잠금 파일이 쌓인다.
200 개가 넘으면 SHM 전송이 죽는데, 증상이 특이하다.

```
Pi 가 보는 노드 수: 27      <- 원격(메인봇) 노드는 보인다
/follower 노드:      0      <- 같은 기계의 자기 노드는 안 보인다
```

로컬 통신만 SHM 을 타기 때문이다. `ros2 lifecycle set/get` 이
`Node not found` 를 내고 Nav2 를 활성화할 수 없게 된다.

증상:

```
RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7022:
open_and_lock_file failed
```

대책 둘. 종료할 때마다 위 `rm -f` 를 돌리거나, UDP 전용으로 못박는다.
팔로워는 후자를 적용해 두었다.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fire_test_logs/fastdds_udp_only.xml
```

### 4-2. ROS setup.bash 는 `set -u` 와 함께 못 쓴다

```
/opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

기동 스크립트에 `set -u` 를 넣으면 소싱 즉시 죽는다. 출력을 `/dev/null`
로 보내고 있으면 **로그가 0 바이트로 남아 원인을 못 찾는다.**

### 4-3. `ros2 topic pub -1` 은 유실된다

단발 발행은 discovery 경쟁에서 져서 상대에게 안 간다. 손으로 쏠 때는
`-r 1 -t 3` 을 쓴다. 노드끼리는 퍼블리셔가 유지되므로 해당 없다.

### 4-4. 도메인

`ROS_DOMAIN_ID=42`. `argos_env.sh` 가 설정한다. 새 셸에서 이걸 안 하면
노드가 하나도 안 보인다.

### 4-5. 지도 파일

- 실내 `maps/argos_lab_imu_v2.*` 는 2026-09-01 18:14 에 덮어써졌다
  (101x152, 91% 미지). 그대로 쓰면 순찰 목표를 못 찾는다. git 원본
  (146x84) 을 꺼내 둔 것이 `~/fire_test_logs/origmap/` 에 있다.
- 실외 `maps/argos_outdoor_imu_v1.*` 는 git 원본 그대로다. 현재 런처가
  이걸 가리킨다.
- 실외 지도는 254 KiB 라 Pi 의 UDP 수신 버퍼(208 KiB)보다 크다. 재전송을
  거쳐 결국 도착하지만 15 s 를 넘긴다. 자세 탐색 도구 대기를 90 s 로
  늘려 두었다.

### 4-6. 라이프사이클 매니저 타임아웃

부하가 있거나 지도가 크면 4 s 안에 활성화를 못 끝내고 매니저가 포기한다.
노드 자체는 살아 있으므로 `ros2 lifecycle set <node> activate` 로 올리면
된다. 기동 스크립트가 이 확인을 포함한다.

---

## 5. 현재 파라미터

| 항목 | 값 | 위치 |
|---|---|---|
| 경보 기준 (MLP 확률) | 0.80 | `fire_prob_threshold`, 원본 0.70 을 실행 중 덮어씀 |
| 텔레그램 쿨다운 | 90 s | `telegram_cooldown_seconds`, 원본 60 s |
| 불 접근 기준 | 0.25 / 0.7 s | `fire_conf`, `confirm_seconds` |
| 스파크 위험 기준 | 0.50 | 주행 `danger_conf_spark`, 인지 `alert_conf_spark` |
| 담배 위험 기준 | 0.50 | 주행 `danger_conf_cigarette_butt`, 인지 `alert_conf_cigarette_butt` |
| 팔로워 도착 반경 | 0.18 m | `arrival_radius_m` |
| 팔로워 복귀 | 켜짐 | `return_home`, 반경 0.30 m |
| 펌프 | `run_sup.sh` 의 `enable_pump` | 현재 true |

**원본 `~/YOLO/new_main_robot_map.py` 는 1 바이트도 수정하지 않았다.**
래퍼(`fire_perception_main.py`)가 실행 중에만 모듈 전역을 바꾼다.

---

## 6. 미해결

1. **팔로워 이격 거리.** `arrival_radius_m: 0.18` 이라 불 좌표 18 cm 까지
   붙는다. 메인봇은 0.60 m 에 멈춘다. 노즐 사거리에 맞춰 접근 방향으로
   목표를 뒤로 물려야 한다. 화재 좌표가 메인봇 근처면
   `RegulatedPurePursuitController detected collision ahead!` 로 막힌다.
2. **MLX90640 이 간헐적이다.** 배선 수정 후 63% 프레임만 유효하다
   (나머지는 `getFrame()` 실패로 0.0). MLP 입력 8 개 중 `temperature`,
   `temp_change` 가 그만큼 흔들린다. `~/YOLO/sensor_diag/sensor_diag.ino`
   를 구우면 0x33 이 버스에 있는지 갈린다.
3. **실외 위치추정이 불안정하다.** 팔로워 유효빔이 450 중 133~145 뿐이고
   가려짐이 31~45% 다. 같은 자리를 탐색할 때마다 다른 답이 나온다.
   지도를 현재 물건 배치로 다시 뜨는 것이 가장 확실하다.
4. **지도 파일 불일치.** 4-5 참고.
