# ARGOS 화재 순찰 주행 (fire_nav_integrated)

Nav2 자동 순찰을 돌다가 화재를 확인하면 접근하고, 좌표를 확정해
팔로워봇에 넘긴 뒤 다시 순찰로 돌아간다.

`fire_nav_patrol.py` 에서 인지 부분을 전부 들어낸 것이다.

## 이 노드가 하지 않는 것

```
YOLO 를 만들지 않는다.        (인지 원본이 1개만 만든다)
카메라를 열지 않는다.          (인지 원본이 1개만 연다)
/dev/ttyACM0 를 열지 않는다.   (인지 원본이 1개만 연다)
MLP 를 읽지 않는다.            (인지가 텔레그램 판단에만 쓴다)
텔레그램을 보내지 않는다.      (인지 원본이 보낸다)
```

장치를 하나도 소유하지 않는다. 인지 결과는 오직 토픽으로만 받는다.
그래서 **인지 프로세스를 먼저 띄워야 한다.**

## 파일 경로

| 파일 | 역할 |
|---|---|
| `~/argos_project/scripts/fire_nav_integrated.py` | 이 문서가 설명하는 주행 노드 |
| `~/argos_project/config/fire_nav_integrated.yaml` | 파라미터 |
| `~/argos_project/scripts/fire_perception_main.py` | 인지 쪽. 토픽으로만 통신 |
| `~/argos_project/docs/PERCEPTION.md` | 인지 쪽 문서 |
| `~/argos_project/maps/argos_outdoor_imu_v1.yaml` | 실외 지도 |
| `~/argos_project/maps/argos_lab_imu_v2.yaml` | 실내 지도 |

백업본은 `scripts/fire_nav_integrated.py.bak_<날짜>` 형식이다.

## 실행

Nav2 가 먼저 떠 있어야 한다. `argos_navigation.launch.py` 가 bringup 을
포함하므로 `argos_bringup` 을 따로 띄우면 안 된다. 모터 시리얼을 두
프로세스가 잡아 실패한다.

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 launch argos_bringup argos_navigation.launch.py map:=$HOME/argos_project/maps/argos_outdoor_imu_v1.yaml nav_cmd_vel_topic:=/cmd_vel_nav_auto
```

```bash
source ~/argos_project/scripts/argos_env.sh && ~/.venv/bin/python ~/argos_project/scripts/fire_nav_integrated.py --ros-args --params-file ~/argos_project/config/fire_nav_integrated.yaml
```

`nav_cmd_vel_topic:=/cmd_vel_nav_auto` 가 중요하다. Nav2 출력을 그리로
빼야 이 노드가 순찰과 화재 접근 중 어느 쪽을 통과시킬지 중재할 수 있다.
이걸 빼면 Nav2 와 화재 접근이 동시에 `/cmd_vel_nav` 에 써서 서로 싸운다.

## 상태 기계

```
NAV_PATROL ──위험 확정──> FIRE_STOP ──> ALERT_WAIT ──> ALIGN ──> APPROACH ──> HOLD
자율 순찰               정지 0.7s    텔레그램 대기   정렬     0.08 m/s    0.6 m 앞
     ▲                                                                      │
     └──────────────────────────────────────────────────────────────────────┘
        전송 확인 / 불 소실 / 15초 경과  →  좌표 발행 후 순찰 복귀
```

| 상태 | 하는 일 | 빠져나가는 조건 |
|---|---|---|
| `NAV_PATROL` | Nav2 에 순찰 목표를 보내고 이동 | 위험 확정 (`confirm_seconds` 연속) |
| `FIRE_STOP` | 정지 | `fire_stop_pause`(0.7s) 경과 |
| `ALERT_WAIT` | 정지 유지, 텔레그램 대기 | 전송 확인 / `alert_wait_timeout`(10s) / 알림 대상 아님 |
| `ALIGN` | 제자리 회전 | `align_tolerance_deg`(8°) 이내 |
| `APPROACH` | 전진 + 미세 조향 | 전방 0.6 m 미만 / 25초 상한 / `realign_deg`(26°) 초과 |
| `HOLD` | 정지, **좌표 확정·발행** | 전송 확인 / 불 소실 5s / `hold_max_seconds`(15s) |

## 접근 제어

**ALIGN** — 전진 0, 회전만. 비례제어 하나다.

```python
angular = clamp(turn_kp * bearing, ±align_w_max)   # kp=1.2, max=0.35
```

**APPROACH** — 전진 속도 고정 `approach_v`(0.08 m/s). 거리에 따른 감속은 없다.

```python
angular = clamp(turn_kp * bearing, ±approach_w_max)   # max=0.25
```

**방위각** — 인지가 주는 `norm_x`(화면 중앙 기준 -1~+1)에서 계산한다.

```
bearing = -bearing_sign * norm_x * camera_hfov/2 + camera_yaw_offset
```

화면 오른쪽(+)은 로봇 기준 시계방향(-)이므로 부호가 뒤집힌다.
왼쪽 불 → `norm_x < 0` → `bearing > 0` → `angular.z > 0` → CCW → 좌회전.
`fire_seeker.py` 현장 시험에서 확인된 관계다.

**거리** — 카메라는 방향만 주고 거리를 못 준다. 정지 판단은 **LiDAR 전방
섹터**(`front_half_angle_deg` 25° 반각)의 최소 거리로 한다. LiDAR 가
176.8° 돌아가 장착돼 있어 TF 로 변환해서 쓴다.

## 접근 대상 클래스

```yaml
approach_classes: ["fire", "cigarette_butt"]
```

이 목록에 있고 방위각을 뽑아낸 클래스만 `ALIGN`/`APPROACH` 로 간다.
조준할 수 없는 것에는 다가갈 수 없기 때문이다.

| 클래스 | 접근 임계값 | 방위각 임계값 |
|---|---|---|
| `fire` | `fire_conf` 0.35 | `bearing_conf_fire` 0.10 |
| `cigarette_butt` | `approach_conf_cigarette_butt` 0.55 | `bearing_conf_cigarette_butt` 0.45 |
| `smoke` | 0.60 (목록에 없어 미사용) | 0.25 |
| `spark` | 0.70 (목록에 없어 미사용) | 0.60 |

꽁초 임계값을 불보다 높게 잡은 이유는 작고 오탐이 잦기 때문이다.

목록에서 빼도 **MLP danger 경로로 정지와 알림은 그대로 동작한다** (접근만
안 함). 담배꽁초·연기는 원본 MLP 가 하나의 위험으로 묶어 판정한다.

## 화재 좌표 확정

`HOLD` 도달 시점에 계산해서 발행한다.

```
fire_yaw = robot_yaw + bearing
fire_x   = robot_x + distance * cos(fire_yaw)
fire_y   = robot_y + distance * sin(fire_yaw)
```

**이 좌표가 정확한 이유** — `HOLD` 는 LiDAR 전방 여유가
`fire_stop_dist`(0.6 m) 이하가 됐을 때 들어온다. 즉 거리가 추정이 아니라
실측으로 확정된다. 단안 카메라의 거리 추정 문제가 사라진다.

접근 상한(25초)으로 멈춘 경우는 0.6 m 보장이 없으므로 실측 전방거리를
쓰고, 그것도 없으면 발행하지 않는다.

**주의** — LiDAR 는 불꽃이 아니라 그 앞의 물체를 잰다. 불이 벽 앞에 있으면
벽까지 0.6 m 에서 멈추고, 불꽃만 공중에 있으면 LiDAR 에 안 잡혀 상한으로
멈춘다. 후자는 좌표 정확도가 떨어진다.

## 토픽

| 토픽 | 방향 | 형식 | 내용 |
|---|---|---|---|
| `/argos/fire_detection` | 구독 | `String` (JSON) | 인지 결과 |
| `/main/fire_target` | 발행 | `PoseStamped` (`map`) | **확정된 화재 좌표. 팔로워봇용** |
| `/argos/fire_alert_request` | 발행 | `String` (JSON) | HOLD 알림 요청 |
| `/argos/fire_episode` | 발행 | `Bool` | 화재 처리 구간 시작/끝 |
| `/amcl_pose` | 구독 | `PoseWithCovarianceStamped` | 로봇 map 좌표 |
| `/initialpose` | 구독 | `PoseWithCovarianceStamped` | 2D Pose Estimate 감지 |
| `/scan` | 구독 | `LaserScan` | 전방 거리 |
| `/map` | 구독 | `OccupancyGrid` | 순찰 목표 샘플링 |
| `/cmd_vel_nav_auto` | 구독 | `Twist` | Nav2 출력 |
| `/cmd_vel_nav` | 발행 | `Twist` | 중재 결과 |

`/main/fire_target` 은 **transient_local** 이라 팔로워봇이 늦게 접속해도
마지막 목표를 받는다.

## cmd_vel 중재

```
Nav2 ──> /cmd_vel_nav_auto ─┐
                            ├─ 중재(mode) ─> /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel
화재 접근 ──> 내부 twist ───┘
```

`set_control_mode(mode)` 로 전환한다.

| mode | 통과시키는 것 |
|---|---|
| `nav` | Nav2 출력 |
| `fire` | 화재 접근 twist |
| `stop` | 아무것도 안 보냄 |

모드 전환 시 즉시 `Twist()` (정지)를 한 번 발행해 이전 명령이 남지 않게 한다.
각 입력에는 타임아웃이 있다 (`nav_cmd_timeout` 0.5s, `fire_cmd_timeout` 0.3s).

## 순찰 목표 선정

`/map` 에서 로봇 주변 `patrol_min_radius`(0.8m) ~ `patrol_max_radius`(2.5m)
범위의 자유 공간을 무작위 샘플링한다. `patrol_clearance`(0.35m) 반경 안에
장애물 셀이 `patrol_free_max`(10) 개를 넘으면 버리고 다시 뽑는다.
`patrol_sample_attempts`(120) 회까지 시도한다.

`require_initial_pose: true` 라서 **RViz 의 2D Pose Estimate 를 찍기 전에는
순찰이 시작되지 않는다.** `/initialpose` 를 받으면 허용된다.

터미널로도 줄 수 있다.

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

## 팔로워봇 연동

메인봇이 **불 근처까지 가서 자기 위치를 알리는** 구조다. 그 좌표로
팔로워봇이 와서 물을 뿌린다.

**메인봇은 반드시 자리를 뜬다.** `hold_max_seconds`(15초)가 그 장치다.
원래는 불이 사라져야만 순찰로 복귀했는데, 그러면 불이 계속 타는 동안
메인봇이 0.6 m 앞에 계속 서 있게 된다. 그 자리가 팔로워봇의 물줄기가
향하는 곳이다.

복귀 시 `danger_stop_cooldown`(30초) 동안 같은 불로 다시 멈추지 않는다.
이게 없으면 돌아서자마자 또 그 불을 보고 멈춰 무한 반복이 된다.

팔로워봇 쪽은 `/main/fire_target` 을 구독해 그 좌표로 이동하면 된다.
**이 부분은 아직 구현되지 않았다** (`~/argos_follow_pi` 쪽 작업).

## 주요 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `approach_on_fire` | `true` | 불에 접근할지. `false` 면 정지·알림만 |
| `hold_max_seconds` | `15.0` | 불이 남아도 이 시간 뒤 복귀 (물줄기 회피) |
| `danger_stop_cooldown` | `30.0` | 복귀 후 같은 위험으로 안 멈추는 시간 |
| `fire_stop_dist` | `0.60` | 접근 정지 거리 [m] |
| `approach_v` | `0.08` | 접근 속도 [m/s] |
| `approach_max_seconds` | `25.0` | 접근 상한 |
| `alert_gate` | `sent` | 텔레그램 대기 방식 (`sent`/`attempted`/`none`) |
| `alert_wait_timeout` | `10.0` | 텔레그램 최대 대기 |
| `require_initial_pose` | `true` | 2D Pose Estimate 필요 |
| `bearing_sign` | `1.0` | 방위각 부호. 반대로 돌면 `-1.0` |
| `output_cmd_topic` | `/cmd_vel_nav` | 중재 출력 |

전체 목록은 `config/fire_nav_integrated.yaml` 에 주석과 함께 있다.

## 성공 시 로그

```
2D Pose Estimate 수신. 자동 순찰을 허용한다.
state=NAV_PATROL scan=OK det=OK front=1.20m hazard=-(0.00) ... patrol_goals=3
위험 확정 [fire]: Nav2 목표 취소, 제어권 획득 (알림 후 approach)
텔레그램 sent (2.3s 대기): 화재 방향 정렬 시작
화재 방향 정렬 완료, 접근 시작
화재 접근 완료: 전방 0.58 m, 정지 유지
화재 좌표 확정: map (+12.72, +0.50)  로봇 (+12.12, +0.50)  전방 0.58 m 방위 +2deg -> /main/fire_target 발행, 알림 요청
텔레그램 sent 확인 (1.2s): 즉시 순찰 복귀, 팔로워봇 진화 공간 확보
```

## 진단

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic echo /main/fire_target
```

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic echo /argos/fire_episode
```

Nav2 lifecycle 상태 (기동 후 30초쯤)

```bash
source ~/argos_project/scripts/argos_env.sh && for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do echo -n "$n: "; ros2 lifecycle get /$n; done
```

`inactive [2]` 가 있으면 활성화가 중간에 멈춘 것이다. 실제로 겪었던 문제다.

```bash
source ~/argos_project/scripts/argos_env.sh && for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do ros2 lifecycle set /$n activate 2>/dev/null; done
```

`map -> odom` 이 나오는지 확인 (AMCL 이 살아있는지)

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 run tf2_ros tf2_echo map odom
```

## 알려진 문제

**회전이 명령보다 약 36% 과구동된다.** gyro 실측 780° vs 명령 575°.
`ALIGN ↔ APPROACH` 를 오가는 진동의 원인이다. 다만 이 수치는 teleop 명령을
0.5초 hold 가정으로 적분한 근사라 실제로는 더 작을 수 있다.
바퀴 고정이 헐거운 기계 문제일 가능성이 있어, 고정한 뒤 재측정해야 한다.

**`amcl.yaml` 의 초기 위치는 실내 지도 기준이다** (`x: 0.128, y: 1.424`).
실외 지도에서는 엉뚱한 곳이므로 반드시 2D Pose Estimate 를 찍어야 한다.

**엔코더 `SERIAL READ` 오류가 간헐적으로 난다.** CH341 USB 케이블/전원 쪽이다.

**launch 를 두 번 띄우면 안 된다.** LiDAR 드라이버 두 개가 `/dev/ttyTHS1` 을
동시에 열어 checksum 오류가 쏟아지고, 모터 시리얼도 충돌한다. 띄우기 전에
확인할 것.

```bash
pgrep -af "ydlidar|argos_base_driver|ekf_node|amcl|scan_normalizer"
```

## 전부 내리기

```bash
pkill -INT -f fire_nav_integrated.py; pkill -INT -f fire_perception_main.py; pkill -INT -f "argos_.*\.launch\.py"; sleep 7; pkill -9 -f ydlidar_ros2_driver_node
```

## 원복

기존 파일을 고치지 않았으므로 새 파일만 지우면 원상복구다.

```bash
rm ~/argos_project/scripts/fire_nav_integrated.py ~/argos_project/config/fire_nav_integrated.yaml
```

`fire_nav_patrol.py` (예전 통합 순찰)는 그대로 남아 있다.
