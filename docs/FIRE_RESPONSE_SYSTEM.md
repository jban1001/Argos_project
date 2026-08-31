# ARGOS 화재 대응 시스템 — 메인봇과 팔로워봇

두 로봇이 나눠 맡는다. 메인봇이 순찰하다 불을 찾아 위치를 확정하고
알리면, 팔로워봇이 그 좌표로 가서 물을 뿌린다.

이 문서는 2026-08-31 기준 **설계와 구현 상태**를 한 곳에 모은 것이다.
개별 구성요소는 아래 문서를 본다.

| 문서 | 범위 |
|---|---|
| `docs/PERCEPTION.md` | 메인봇 화재 인지 (YOLO/MLP/텔레그램) |
| `docs/FIRE_NAV.md` | 메인봇 순찰·접근 주행 |
| 이 문서 | 두 로봇을 잇는 전체 그림 |
| Pi `~/lidar_overlay_ws/docs/follower_system_2026-08-31.md` | 팔로워 내부 |

---

## 1. 전체 흐름

```
[ 메인봇 · Jetson Orin Nano · 192.168.0.18 ]

  카메라 ─► YOLO/MLP ─► /argos/fire_detection ─┐
                                                │
  LiDAR ──► /scan ──┐                           │
  IMU ────► EKF ────┴─► /odom ──► Nav2 ─────────┤
                                                ▼
                                    fire_nav_integrated
                                    순찰 → 정지 → 접근 → HOLD
                                                │
                     ┌──────────────────────────┼──────────────────┐
                     ▼                          ▼                  ▼
              텔레그램 알림          /main/fire_target      /fire/dispatch
              (사람에게)             (PoseStamped)          (JSON, 팔로워)
                                                                   │
─────────────────────────────────────────────────────────────────  │
                                                                   │
[ 팔로워봇 · Raspberry Pi 5 · argos2026.local · 192.168.0.107 ]     │
                                                                   ▼
  LiDAR ─► mask ─► scan_odom ─┐                          fire_supervisor
  IMU(MCU) ───────────────────┴─► EKF ─► follower_odom   IDLE
                                            │            → WAIT_CLEARANCE
  메인봇 /map ─► AMCL ─► map→follower_odom  │            → NAVIGATING
                                            ▼            → SETTLING
                                          Nav2           → SPRAYING
                                            │            → COMPLETE
                                            ▼
                                    cmd_vel_bridge ─► serial_bridge ─► Mega
                                                                        │
                                                            모터 + 펌프(D9)
```

두 로봇은 같은 `ROS_DOMAIN_ID=42`, 같은 `/map` 을 쓴다.
지도가 같아야 좌표가 통한다.

---

## 2. 왜 이렇게 나눴는가

**메인봇이 불에 다가가서 자기 위치를 알린다.**

단안 카메라는 방위각만 주고 거리를 못 준다. 그런데 메인봇이 LiDAR 전방
여유 0.6 m 까지 접근해 멈추면(`HOLD`), 그 순간 "불이 정면 0.6 m 앞" 이
실측으로 확정된다. **메인봇의 현재 위치가 곧 불 위치의 대용**이 되므로
거리 추정 문제가 사라진다.

**그리고 메인봇은 반드시 자리를 뜬다.**

팔로워봇이 그 지점에 물을 뿌린다. 메인봇이 0.6 m 앞에 서 있으면 그대로
맞는다. 그래서 `hold_max_seconds`(15초)가 지나면 불이 아직 보여도 순찰로
돌아간다. 원래는 불이 사라져야만 복귀했는데 그러면 계속 서 있게 된다.

**복귀하는 순간이 팔로워에게 "이제 와도 된다" 고 알릴 때다.**

---

## 3. 인계 규약 — `/fire/dispatch`

유일한 출처는 팔로워의 `follower_fire_control/protocol.py` 다.

**형식** — `std_msgs/String` 에 JSON. 필드 7개 정확히, 추가·누락 모두 거부.

```json
{"schema": 1,
 "mission_id": "argos-1788184618001",
 "frame_id": "map",
 "x": -0.5, "y": 0.3, "yaw": 0.0,
 "main_cleared": false}
```

| 필드 | 제약 |
|---|---|
| `schema` | 반드시 `1` |
| `mission_id` | `[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}` |
| `frame_id` | 반드시 `"map"` |
| `x`, `y`, `yaw` | 유한한 수. yaw 는 `[-pi, pi]` 로 정규화됨 |
| `main_cleared` | JSON boolean |

**두 번 보낸다. 좌표는 완전히 같아야 한다.**

```
HOLD 도달   main_cleared=false  → 팔로워 WAIT_CLEARANCE (대기)
순찰 복귀   main_cleared=true   → 팔로워 NAVIGATING (출발)
```

같은 `mission_id` 로 다른 좌표를 보내면
`same mission_id cannot change target` 으로 거부된다. 1e-6 이내로 비교하므로
메인봇은 첫 지령을 저장해 두었다가 그대로 재사용한다.

`main_cleared` 를 `false → true` 로 올리는 것만 허용된다.

---

## 4. 메인봇 구현

### 4.1 파일

| 파일 | 역할 |
|---|---|
| `scripts/fire_perception_main.py` | 인지팀 원본을 감싸 ROS 로 중계 |
| `scripts/fire_nav_integrated.py` | 순찰·접근·좌표 확정·지령 발행 |
| `config/fire_nav_integrated.yaml` | 주행 파라미터 |
| `ros2_ws/src/argos_bringup/launch/argos_navigation.launch.py` | Nav2 + bringup |

### 4.2 상태 기계

```
NAV_PATROL ──위험 확정──> FIRE_STOP ──> ALERT_WAIT ──> ALIGN ──> APPROACH ──> HOLD
     ▲                                                                        │
     └────────── 전송 확인 / 불 소실 / hold_max_seconds(15초) ─────────────────┘
```

`HOLD` 진입 시:
- 화재 map 좌표 확정 `fire = robot + distance x (cos, sin)(robot_yaw + bearing)`
- `/main/fire_target` 발행 (`PoseStamped`, transient_local)
- `/fire/dispatch` 발행 (`main_cleared=false`)

`NAV_PATROL` 복귀 시:
- `/fire/dispatch` 재발행 (`main_cleared=true`) ← 팔로워 출발 신호
- `/argos/fire_episode` = false
- `danger_stop_cooldown`(30초) 동안 같은 불로 다시 안 멈춤

### 4.3 접근 대상

```yaml
approach_classes: ["fire", "cigarette_butt"]
fire_conf: 0.35
approach_conf_cigarette_butt: 0.55
```

꺼지지 않은 꽁초도 발화원이라 불과 같이 처리한다. 꽁초는 작고 오탐이 잦아
임계값을 높인다. 목록에서 빼도 MLP danger 경로로 정지·알림은 동작한다.

### 4.4 텔레그램

원본이 불을 발견한 즉시 보낸다. **접근 실패해도 알림은 이미 나갔다** —
장애물에 막혀 통보가 안 가는 것이 최악이라 안전망으로 남겼다.

지도 사진은 건물 도면과 실제 SLAM 지도를 **한 장으로 합쳐** 보낸다.
도면(`nabil_map.png`)은 SLAM 지도를 따라 그린 것이 아니라 벽 일치율이 8% 고
위치가 2~3 m 어긋난다. 자동 정렬을 시도했으나 8.0% → 8.3% 로 개선되지 않았다.

접근하는 동안의 중복 알림은 `/argos/fire_episode` 신호로 막되,
**한 화재 건에 최소 1회는 통과**시킨다.

---

## 5. 팔로워봇 구현

### 5.1 프레임 사슬

```
map → follower_odom → follower_base_link → follower_laser_frame
                              └─────────► follower_imu_link
```

- `map → follower_odom` : AMCL (메인봇의 `/map` 을 구독. 자체 map_server 없음)
- `follower_odom → follower_base_link` : EKF (TF 소유자)
- `base_link → laser_frame` : 실측 (2026-08-30)
  `x=-0.043, y=+0.018, z=0.120, roll=180deg, yaw=-2.77deg`
  roll=180 은 거울상이다 — 라이다가 거꾸로 달렸다.

### 5.2 미션 상태

```
IDLE → WAIT_CLEARANCE → NAVIGATING → SETTLING → SPRAYING → COMPLETE
                                                              ↓ 실패 시
                                                            FAILED
```

주행은 Nav2 가 한다 (`navigator: "nav2"`). 내장 점 제어기는 장애물 없을 때용.

### 5.3 안전 장치

| 장치 | 값 | 뜻 |
|---|---|---|
| `enable_motion` | 기본 `false` | 끄면 목표조차 안 나간다 |
| `enable_pump` | 기본 `false` | 끄면 펌프 명령 안 나간다 |
| `front_obstacle_distance_m` | 0.45 | 전방 60도 안에 이보다 가까우면 비상정지 |
| `mission_timeout_s` | **120** | 이 안에 못 끝내면 실패 |
| `max_spray_duration_s` | 10 | 원격 지령이 늘릴 수 없다 |
| `pose_max_age_s` | 0.5 | 자세가 오래되면 fail-closed |

### 5.4 MCU 펌웨어

`firmware/followingbot_mega_fire/followingbot_mega_fire.ino` (v2.1)

```
FW,followingbot_mega_fire,2.1
CFG,ACCEL_FS_G,2,GYRO_FS_DPS,250,IMU_RATE_HZ,200,BAUD,230400
Commands: F B L R S, C,throttle,yaw_rate, M,left_pwm,right_pwm, I,0|1, D, P,0|1
PUMP_PIN = 9, PUMP_ON_LEVEL = HIGH
```

**이 펌웨어가 아니면 IMU·모터·펌프가 전부 죽는다.** 2026-08-31 에 실제로
`pump_servo_test.ino` 가 올라가 있어 MCU 가 완전히 침묵했다. §8 참고.

---

## 6. 검증 상태

### 6.1 확인된 것

| 항목 | 근거 |
|---|---|
| 메인봇 `/main/fire_target` 발행 | Jetson 내부 구독으로 좌표 확인 |
| 메인봇 → 팔로워 ROS 통신 | Pi 에서 `/main/fire_target` 수신 확인 |
| dispatch 규약 적합성 | 팔로워의 실제 `parse_dispatch()` 로 파싱 성공 |
| **`IDLE → WAIT_CLEARANCE`** | `main_cleared=false` 전송 → 상태 전이 확인 |
| **`WAIT_CLEARANCE → NAVIGATING`** | `main_cleared=true` 전송 → 전이 + 모터명령 산출 |
| 팔로워 AMCL | 프리체크 통과, 초기자세 자동탐색 벽뚫음 0.0% |
| 팔로워 IMU | 199.8 Hz (펌웨어 복구 후) |
| 메인봇 불 감지 → 접근 → 텔레그램 | 실물 시험 통과 |

### 6.2 확인 안 된 것

| 항목 | 이유 |
|---|---|
| **팔로워 실주행** | 첫 시도에서 뒤로 갔다. §8 참고 |
| 펌프 실제 분사 | 물 빼고 dry-run 만 |
| 메인봇 실화재 → 자동 dispatch 전체 사슬 | 수동 지령으로만 시험 |
| 지도 결합 사진이 텔레그램에 오는지 | 오프라인 렌더링만 확인 |
| 알림 게이트 실동작 | 시뮬레이션만 확인 |
| 담배꽁초 실제 confidence | YOLO 가 꽁초에 얼마를 주는지 미측정 |

---

## 7. 운용 절차

### 7.1 메인봇 (Jetson)

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 launch argos_bringup argos_navigation.launch.py map:=$HOME/argos_project/maps/argos_outdoor_imu_v1.yaml nav_cmd_vel_topic:=/cmd_vel_nav_auto
```

```bash
source ~/argos_project/scripts/argos_env.sh && ~/.venv/bin/python ~/argos_project/scripts/fire_perception_main.py 2>&1 | grep --line-buffered -v "Corrupt JPEG data"
```

```bash
source ~/argos_project/scripts/argos_env.sh && ~/.venv/bin/python ~/argos_project/scripts/fire_nav_integrated.py --ros-args --params-file ~/argos_project/config/fire_nav_integrated.yaml
```

RViz 에서 2D Pose Estimate 로 초기 위치를 찍어야 순찰이 시작된다.

### 7.2 팔로워봇 (Pi) — 메인봇 `/map` 이 먼저 떠 있어야 한다

```bash
bash ~/lidar_overlay_ws/tools/start_localization_stack.sh
```

```bash
FOLLOWER_PEER_HOST=192.168.0.18 bash ~/lidar_overlay_ws/tools/start_follower_amcl.sh --start
```

```bash
python3 ~/lidar_overlay_ws/tools/scan_map_search.py --publish
```

```bash
ros2 launch ~/lidar_overlay_ws/launch/follower_nav2.launch.py
```

```bash
ros2 launch ~/lidar_overlay_ws/launch/follower_fire.launch.py enable_motion:=false enable_pump:=false start_serial_bridge:=false
```

`start_serial_bridge:=false` 는 위 1번 스크립트가 이미 브리지를 띄우기
때문이다. 두 개가 뜨면 `/dev/ttyACM0` 을 두고 싸운다.

### 7.3 수동 지령 (시험용)

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic pub /fire/dispatch std_msgs/msg/String "{data: '{\"schema\":1,\"mission_id\":\"test-001\",\"frame_id\":\"map\",\"x\":-0.5,\"y\":0.3,\"yaw\":0.0,\"main_cleared\":false}'}" -r 2 --keep-alive 4
```

`main_cleared` 를 `true` 로 바꿔 한 번 더 보내면 출발한다.
**`--keep-alive` 없이 `--once` 만 쓰면 discovery 경합으로 전달이 안 될 수 있다.**

### 7.4 관찰

```bash
ros2 topic echo /follower/fire/status
```

```bash
ros2 topic echo /main/fire_target --qos-durability transient_local --qos-reliability reliable
```

### 7.5 비상 정지

```bash
ros2 topic pub --once /fire/reset std_msgs/msg/Empty "{}"
```

```bash
pkill -f fire_supervisor_node; pkill -f follow_controller_node
```

---

## 8. 남은 문제

### 8.1 팔로워가 뒤로 갔다 — 미해결

2026-08-31 첫 실주행에서 로봇이 **뒤로 움직였고**, 사용자가 손으로 들어
옮겨야 했다.

의심 지점 두 가지. **어느 쪽인지 아직 못 가렸다.**

1. `follower_fire.yaml` 의 `invert_drive: true` 가 지금 하드웨어와 맞는지
   검증된 적이 없다. 팔로워 문서 §7.2 에도 `[미검증]` 으로 남아 있다.
2. 사람이 로봇을 들어 옮기면 AMCL 추정과 실제가 어긋난다. 그러면 로봇은
   "아직 목표에 못 갔다" 고 판단해 계속 간다.

**다음에 시험할 때는 손대지 말고, 이상하면 즉시 정지시킬 것.**
`invert_drive` 를 먼저 낮은 속도로 단독 검증하는 편이 안전하다.

### 8.2 `mission_timeout_s: 120` 이 짧을 수 있다

1차와 2차 지령 사이에 시간을 쓰면 미션이 타임아웃으로 죽는다. 실제로
시험 중 한 번 그렇게 실패했다. 메인봇이 HOLD 에서 최대 15초 머물고 바로
clearance 를 보내므로 정상 운용에서는 문제없지만, **팔로워가 목표까지
가는 시간도 이 120초 안에 들어가야 한다.**

### 8.3 감독기 비상정지와 Nav2 회피가 겹친다

`front_obstacle_distance_m: 0.45` 는 전방 60도 안에 그보다 가까운 것이
있으면 미션을 실패시킨다. Nav2 는 장애물을 0.45 m 안쪽으로 스쳐 지나가는
경로를 만들 수 있다. **어느 쪽을 양보할지 아직 정하지 않았다.**

### 8.4 메인봇 회전 과구동

명령보다 약 36% 더 돈다 (gyro 실측 780도 vs 명령 575도). 다만 이 수치는
teleop 명령을 0.5초 hold 로 가정해 적분한 근사다. 바퀴 고정이 헐거운
기계 문제일 가능성이 있어 고정 후 재측정이 필요하다.

### 8.5 지도를 바꾸면 양쪽을 같이 바꿔야 한다

팔로워는 자체 map_server 가 없고 메인봇의 `/map` 을 그대로 쓴다. 메인봇에서
지도를 바꾸면 팔로워 AMCL 도 새 지도로 다시 초기자세를 잡아야 한다.

실외 시험은 `argos_outdoor_imu_v1` (28.1 x 23.2 m) 을 쓴다.
실내는 `argos_lab_imu_v2` (7.3 x 4.2 m).

---

## 9. 네트워크

| 기기 | 주소 |
|---|---|
| 메인봇 | `192.168.0.18` (`odyssey`) |
| 팔로워봇 | `192.168.0.107` (`argos2026.local`) |
| 노트북 | `192.168.0.218` (WSL) |

`ROS_DOMAIN_ID=42` 공통.

**mDNS 가 안 먹는다.** 팔로워의 `env.sh` 가 `odyssey.local` 을 해석하지
못하므로 `FOLLOWER_PEER_HOST=192.168.0.18` 로 덮어쓴다.

**WSL 은 방화벽 때문에 자주 끊긴다.** mirrored 네트워크로 바꾸고 Hyper-V
방화벽 inbound 를 허용해야 하는데, WSL 재시작이나 재부팅으로 풀린다.
`.wslconfig` 에 `firewall=false` 를 넣는 쪽이 확실하다. 팔로워 Pi 는 이런
계층이 없어 그냥 붙는다.

**`ros2` CLI 가 노드를 못 볼 때가 잦다.** 데이터는 흐르는데 CLI 만 못 보는
경우다. 이럴 때는 daemon 을 다시 띄운다.

```bash
ros2 daemon stop && ros2 daemon start
```
