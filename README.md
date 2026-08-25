# ARGOS ROS 2 자율주행

Jetson Orin Nano와 ROS 2 Jazzy를 사용하는 ARGOS 실물 이동로봇의 고정맵 기반 자율주행 프로젝트이다.

이 문서는 2026-08-25 기준 **실물 로봇 실행 절차와 기준 구성(baseline)** 을 설명한다.
현재 활성 구성은 다음과 같다.

- 지도: 실내 `argos_lab.yaml` 또는 실외 `argos_outdoor_v1.yaml` 고정맵
- 위치 추정: AMCL
- 전역 경로 계획: Smac Planner State Lattice
- 경로 추종: Regulated Pure Pursuit(RPP)
- 작업 조정: Nav2 Behavior Tree(BT)
- 구동: 차동/스키드 스티어, 제자리 회전 가능
- 시뮬레이션이 아닌 실제 모터·엔코더·YDLIDAR 사용
- 선택 기능: 임의 목표 순찰 → 화재 감지 → Nav2 취소 → 화재 방향 접근

> **중요:** 이 구성에서는 주행 중 지도를 다시 만들거나 갱신하지 않는다. `slam_toolbox`와 localization/navigation launch를 동시에 실행하면 안 된다. 고정맵의 장애물 레이어와 AMCL 위치 추정만 실시간으로 갱신한다.

## 가장 빠른 Navigation 실행

아래 명령은 모두 Jetson(`odyssey`)에서 실행한다. 현재 접속 예시는 다음과 같다.

```bash
ssh -Y odyssey@192.168.0.18
```

IP는 공유기/DHCP에 따라 바뀔 수 있다. 접속이 안 되면 Jetson에서 `hostname -I`로 주소를 다시 확인한다.

### 1. 환경 불러오기

새 터미널마다 먼저 실행한다.

```bash
cd ~/argos_project
source scripts/argos_env.sh
```

`scripts/argos_env.sh`는 ROS 2 Jazzy, 프로젝트 workspace, YDLIDAR workspace를 불러오고 `ROS_DOMAIN_ID=42`를 설정한다. 여러 터미널을 사용할 때 모든 터미널의 domain이 같아야 한다.

코드를 처음 받았거나 ROS package를 변경했다면 한 번 빌드한다.

```bash
cd ~/argos_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 실내 또는 실외 Navigation 실행

한 번에 하나만 선택한다.

실내맵:

```bash
ros2 launch argos_bringup argos_navigation.launch.py \
  map:=$HOME/argos_project/maps/argos_lab.yaml
```

실외맵:

```bash
ros2 launch argos_bringup argos_navigation.launch.py \
  map:=$HOME/argos_project/maps/argos_outdoor_v1.yaml
```

이 launch 하나가 모터 드라이버, wheel odometry, LiDAR, map server, AMCL, planner, controller, BT navigator와 velocity smoother를 모두 실행한다. 별도 터미널에서 `argos_bringup.launch.py`, SLAM 또는 다른 Navigation launch를 중복 실행하지 않는다.

### 3. RViz에서 초기 위치와 목표 지정

`ssh -Y` 접속에서 launch에 RViz가 포함되지 않은 경우 새 터미널에서 다음을 실행한다.

```bash
cd ~/argos_project
source scripts/argos_env.sh
rviz2 -d ros2_ws/src/argos_bringup/rviz/argos_navigation.rviz
```

1. RViz의 Fixed Frame이 `map`인지 확인한다.
2. **2D Pose Estimate**로 실제 로봇 위치와 전방 방향을 지정한다.
3. LaserScan 점이 지도 벽/장애물과 대략 겹치는지 확인한다.
4. **Nav2 Goal**로 도착 위치와 방향을 지정한다.
5. `/plan`이 만들어지고 로봇이 회전 후 경로를 따라가는지 확인한다.

초기 위치가 틀린 상태에서 Goal을 보내면 경로가 이상하거나 잠깐 움직인 뒤 abort될 수 있다.

### 4. 자동 순찰 + 화재 접근 실행

일반 Navigation을 먼저 종료한 뒤 실행한다. 기본 지도는 실외맵이다.

```bash
cd ~/argos_project
source scripts/argos_env.sh
ros2 launch argos_bringup argos_fire_patrol.launch.py use_rviz:=true
```

실내맵으로 실행하려면:

```bash
ros2 launch argos_bringup argos_fire_patrol.launch.py \
  map:=$HOME/argos_project/maps/argos_lab.yaml \
  use_rviz:=true
```

실행 후 반드시 RViz에서 **2D Pose Estimate**를 한 번 지정해야 순찰을 시작한다. 동작 순서는 다음과 같다.

1. 저장맵의 빈 공간에서 임의 순찰 목표 선택
2. Nav2로 목표까지 이동하고 다음 목표 반복
3. 카메라에서 화재가 연속 확인되면 현재 Nav2 goal 취소
4. 화재가 화면 중앙에 오도록 제자리 정렬
5. 전방 LiDAR 거리를 확인하며 최대 `0.08 m/s`로 접근
6. 약 `0.60 m` 앞에서 정지하고 HOLD

순찰 중 속도 흐름은 아래처럼 분리되어 있다.

```text
Nav2 /cmd_vel_nav_auto ─┐
                        ├─ fire_nav_patrol ─> /cmd_vel_nav
화재 접근 /cmd_vel_fire ┘                         │
                                      velocity_smoother
                                                │
                                            /cmd_vel
                                                │
                                       argos_base_driver
```

`~/YOLO` 원본은 수정하지 않는다. 사용자가 만든 `~/YOLO/YOLO_BACK`의 모델과 `telegram_alert.py`만 사용한다.

텔레그램 기본 경보 조건은 `new_main.py`의 센서+MLP 방식을 따른다. 다만
현재 `fire_mlp.pkl`에는 실제 spark 학습자료가 부족하므로 ARGOS 순찰에서는
기본적으로 MLP 입력의 `spark_conf`를 실제 값의 10%로 축소한다.

- Arduino 온도/가스 센서 연결 정상
- MLP 위험확률 70% 이상
- 위 조건이 1초 연속 유지
- 메시지와 감지 사진 전송
- 감지 순간의 고정맵에 빨간 로봇 아이콘과 주황색 화재 방향선을 표시해 별도 전송
- 같은 위험이 계속되면 60초 간격으로 재전송
- 메시지에 YOLO 클래스 confidence, 온도, 가스, 로봇 상태와 map 위치 포함

설정은 `config/fire_nav_patrol.yaml`의 `telegram_*` 항목에서 조정한다. 기본 `telegram_gate: mlp`가 최종 운용 설정이다. 현재 MLP의 현장 오탐을 확인하는 시연 단계에서만 다음처럼 `yolo`로 바꿀 수 있다.

`telegram_mlp_spark_weight: 0.10`이어도 YOLO 화면의 spark 박스/confidence와
텔레그램 메시지의 spark 탐지값은 그대로 남는다. 오직 MLP 경보확률을 계산할
때만 spark 값을 10%로 줄인다. 예를 들어 화면의 `0.20`은 MLP에 `0.02`로
입력된다. 나중에 실제 spark 양성·음성 데이터를 포함해 MLP를 다시 학습한
뒤에는 이 값을 `1.0`으로 바꿀 수 있다.

```yaml
telegram_gate: yolo
telegram_yolo_conf: 0.20
```

접근 동작 자체는 MLP 단독 오탐으로 로봇이 움직이지 않도록 YOLO의 실제 `fire` bbox와 방위각을 사용한다. 텔레그램 자격정보는 환경변수, `YOLO_BACK/telegram_config.json`, 기존 백업 설정 순으로 읽으며 토큰을 로그에 출력하지 않는다.

지도 사진의 빨간 삼각형은 경보가 확정된 순간의 AMCL 로봇 위치와 전방 방향이다. 주황색 화살표는 `로봇 yaw + 카메라 fire bearing`으로 계산한 방향만 나타낸다. 화재 거리 센서가 없으므로 화살표 길이는 실제 거리나 화재 좌표를 뜻하지 않는다. `/map`이나 `/amcl_pose`가 아직 없다면 카메라 경보는 보내되 지도 사진은 생략한다.

현재 MLP가 충분히 학습되지 않았다면 오탐이 생길 수 있으므로, 시연 전에는 사람이 비상 정지할 수 있는 거리에서 낮은 속도로 시험한다.

### 5. 수동주행(데이터 수집용)

Nav2와 화재 순찰을 모두 종료한다. 터미널 1:

```bash
cd ~/argos_project
source scripts/argos_env.sh
ros2 launch argos_bringup argos_bringup.launch.py
```

터미널 2:

```bash
cd ~/argos_project
source scripts/argos_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.08 -p turn:=0.20
```

`i` 전진, `,` 후진, `j`/`l` 회전, `k` 정지이다. 명령을 웹 문서에서 복사할 때 `~`, `_`, `--` 앞에 역슬래시를 붙이지 않는다.

연결 확인:

```bash
ros2 topic info /cmd_vel
```

teleop 실행 중 `Publisher count: 1`, base driver 실행 중 `Subscription count: 1`이어야 한다.

### 6. 실외맵 다시 만들기

Navigation/AMCL/화재 순찰을 모두 종료하고 SLAM만 실행한다.

```bash
cd ~/argos_project
source scripts/argos_env.sh
ros2 launch argos_bringup argos_slam.launch.py
```

다른 터미널에서 수동주행하거나 `scripts/mapping_drive.py`를 사용해 천천히 한 바퀴 돌며 겹치는 구간을 충분히 만든다. 맵 저장:

```bash
cd ~/argos_project
source scripts/argos_env.sh
ros2 run nav2_map_server map_saver_cli \
  -f maps/argos_outdoor_v1
```

다음 두 파일이 생기면 저장된 것이다.

```bash
ls -lh maps/argos_outdoor_v1.yaml maps/argos_outdoor_v1.pgm
```

저장 후 SLAM을 `Ctrl+C`로 종료하고 위의 실외 Navigation 명령으로 다시 실행한다.

## 현재 상태

| 항목 | 상태 | 비고 |
|---|---|---|
| Jetson OS | Ubuntu 24.04.4 LTS | aarch64, Jetson 커널 |
| ROS 2 | Jazzy | 실제 하드웨어 구동 |
| 고정맵 | 사용 중 | 실내/실외 2개, 0.05 m/cell |
| Localization | 사용 중 | AMCL, differential motion model |
| Planner | 사용 중 | `SmacPlannerLattice` |
| Controller | 사용 중 | `RegulatedPurePursuitController` |
| BT | 사용 중 | `config/navigate_to_pose_smart.xml` |
| Nav2 lifecycle | 활성화 확인 | controller/planner/BT 모두 `active [3]` |
| LiDAR | 입력 확인 | 복원 직후 약 3.66 Hz |
| 실주행 | 확인 | 실외맵에서 RViz Nav2 Goal 구동 확인 |

MPPI 관련 파일은 비교 및 향후 실험용으로 남아 있지만 현재 launch와 `config/nav2_params.yaml`에서는 사용하지 않는다.

## 시스템 구조

```text
실제 센서/구동 계층
  YDLIDAR ─> /scan_raw ─> scan_normalizer ─> /scan
  Encoder ─> /wheel_ticks/* ─> wheel_odometry_node ─> /odom + odom->base_link
  /cmd_vel ─> argos_base_driver ─> Arduino/모터

고정맵 위치 추정
  argos_lab.yaml ─> map_server ─> /map
  /map + /scan + /odom ─> AMCL ─> map->odom

Nav2
  RViz Nav2 Goal
       └─> BT Navigator
            ├─> SmacPlannerLattice ─> /plan
            ├─> RPP ─> /cmd_vel_nav
            └─> recovery behaviors
  /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel ─> base driver
```

### TF 구조

```text
map ──AMCL──> odom ──wheel_odometry_node──> base_link ──static TF──> laser_frame
```

TF 발행 주체를 중복 실행하면 안 된다.

- `map -> odom`: AMCL만 발행
- `odom -> base_link`: wheel odometry만 발행
- `base_link -> laser_frame`: `argos_bringup.launch.py`의 static transform publisher가 발행
- SLAM을 동시에 실행하면 `map -> odom`이 충돌하여 localization과 Nav2가 깨질 수 있음

## 각 구성요소의 책임

### 고정맵과 AMCL

- `maps/argos_lab.yaml`과 `maps/argos_lab.pgm`이 환경의 기준 지도이다.
- `map_server`는 이 지도를 `/map`으로 발행한다.
- AMCL은 `/scan`과 wheel odometry를 사용해 고정맵 기준 로봇 위치를 보정한다.
- 시작 위치 기본값은 `config/amcl.yaml`의 `(x=0.128, y=1.424, yaw=-1.908)`이다.
- 로봇이 실제로 이 위치에 없으면 RViz의 **2D Pose Estimate**로 먼저 현재 자세를 지정해야 한다.

AMCL이 위치를 갱신하는 것과 지도를 갱신하는 것은 다르다. 이 구성에서 AMCL은 `map -> odom` 보정값만 갱신하며 고정맵 이미지 자체는 변경하지 않는다.

### State Lattice 전역 플래너

RViz에서 보이는 전역 경로 `/plan`은 RPP가 아니라 `SmacPlannerLattice`가 만든다.

- motion primitive: `config/argos_lattice.json`
- 모델: differential/skid-steer
- 맵과 lattice 해상도: 0.05 m
- heading 수: 16
- 제자리 회전 primitive 포함
- `allow_reverse_expansion: false`
- 계획 제한 시간: 5초
- 목표 허용 오차: 0.25 m
- 경로 smoothing 활성화

따라서 빨간 전역 경로 자체가 비효율적이면 RPP를 변경할 것이 아니라 lattice, planner penalty, costmap 또는 smoother를 조정해야 한다.

### RPP 경로 추종기

RPP는 State Lattice가 만든 경로를 실제 로봇이 따라가도록 속도 명령을 생성한다.

주요 기준값은 다음과 같다.

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `desired_linear_vel` | 0.12 m/s | 실측 최대 0.158 m/s보다 낮게 운용 |
| lookahead | 0.25~0.70 m | 속도에 따라 가변, 기본 0.40 m |
| `transform_tolerance` | 1.0 s | 느린 LiDAR/TF 지연 고려 |
| 충돌 예측 시간 | 1.5 s | carrot까지 충돌 검사 |
| 최소 곡률 감속 속도 | 0.04 m/s | 급곡선에서 정지에 가까워지는 현상 제한 |
| 시작 방향 정렬 | 활성화 | 0.6 rad 이상이면 제자리 회전 |
| 회전 속도 | 0.4 rad/s | 실측 한계 이내 |
| 후진 추종 | 비활성화 | `allow_reversing: false` |
| 목표 허용 오차 | 위치 0.15 m, 방향 0.25 rad | goal checker 기준 |

RPP는 고정된 운동학적 경로를 일관되게 추종하는 데 유리하지만, MPPI처럼 경로에서 적극적으로 이탈해 동적 장애물을 우회하는 방식은 아니다.

### Behavior Tree

활성 BT는 `config/navigate_to_pose_smart.xml`이다. 목표를 한 번 계산하고 끝내는 단순 BT가 아니라 조건부 재계획과 recovery를 포함한다.

정상 주행 흐름:

1. `GridBased`라는 ID의 State Lattice 플래너로 경로 계산
2. `FollowPath`라는 ID의 RPP로 경로 추종
3. 2 Hz로 경로 상태를 확인
4. 다음 중 하나이면 전역 경로 재계산
   - 기존 경로가 생성된 지 10초가 지남
   - RViz goal이 변경됨
   - 현재 경로가 costmap 기준 유효하지 않음
5. 그 외에는 기존 경로를 계속 추종

실패 처리:

- 경로 계산 실패: global costmap을 지운 뒤 한 번 재시도
- 경로 추종 실패: local costmap을 지운 뒤 한 번 재시도
- 상위 navigation recovery: 최대 6회
- recovery 순환: local/global costmap clear → spin → 5초 wait → 0.30 m backup

이 BT 때문에 `/plan`이 약 10초 단위로 바뀔 수 있다. 이는 컨트롤러가 임의로 경로를 다시 만드는 현상이 아니라 BT가 플래너를 다시 호출하는 동작이다. 불필요한 경로 변경이 계속 문제라면 BT의 재계획 정책을 먼저 A/B 테스트해야 한다.

### Costmap과 footprint

- footprint: 길이 0.28 m × 폭 0.53 m의 직사각형
- inflation radius: 0.45 m
- local costmap: `odom` 기준 3 m × 3 m rolling window, 5 Hz 갱신
- global costmap: `map` 기준, static + LiDAR obstacle + inflation
- 장애물 marking/clearing: `/scan` 사용
- obstacle 최대 거리: 5 m
- raytrace 최대 거리: 6 m

현재 `base_link`가 차체 앞뒤 중앙에 있다는 가정이 들어 있다. 구동축 중심이 실제로 치우쳐 있다면 footprint x 좌표를 수정해야 한다.

## 주요 토픽과 프레임

| 이름 | 형식/역할 | 발행 주체 |
|---|---|---|
| `/map` | 고정 OccupancyGrid | map_server |
| `/scan_raw` | 원본 LaserScan | YDLIDAR driver |
| `/scan` | 1440-bin 정규화 LaserScan | scan_normalizer |
| `/odom` | wheel odometry | wheel_odometry_node |
| `/plan` | 전역 경로 | planner_server |
| `/cmd_vel_nav` | Nav2 원시 속도 명령 | controller/behavior server |
| `/cmd_vel` | smoothing 후 모터 명령 | velocity_smoother |
| `/particle_cloud` | AMCL 파티클 | AMCL |
| `/tf`, `/tf_static` | 좌표계 변환 | AMCL/odometry/static TF |

## 프로젝트 파일

```text
~/argos_project/
├── config/
│   ├── amcl.yaml
│   ├── argos_lattice.json
│   ├── base_driver.yaml
│   ├── fire_nav_patrol.yaml         # 순찰/화재 접근 설정
│   ├── lidar_mount.yaml
│   ├── navigate_to_pose_smart.xml   # 현재 활성 BT
│   ├── navigate_to_pose_once.xml    # 재계획 없는 진단용 BT
│   ├── nav2_params.yaml             # 현재 활성 RPP 설정
│   └── odometry_calibration.yaml
├── maps/
│   ├── argos_lab.yaml
│   ├── argos_lab.pgm
│   ├── argos_outdoor_v1.yaml
│   └── argos_outdoor_v1.pgm
├── ros2_ws/src/
│   ├── argos_bringup/
│   │   └── launch/
│   │       ├── argos_bringup.launch.py
│   │       ├── argos_fire_patrol.launch.py
│   │       ├── argos_localization.launch.py
│   │       └── argos_navigation.launch.py
│   └── argos_odometry/
├── scripts/
│   ├── fire_nav_patrol.py           # Nav2 순찰/화재 접근/속도 중재
│   ├── fire_seeker.py               # 카메라 화재 감지와 접근 로직
│   └── argos_env.sh                 # 공통 ROS 환경
├── docs/
└── README.md
```

Windows/WSL의 RViz 보조 파일은 다음 위치에 있다.

```text
C:\Users\박현성\OneDrive\Documents\ChatGPT\argos\
├── config/
│   ├── argos_navigation.rviz
│   └── fastdds_wsl.xml
└── scripts/
    └── start_wsl_rviz.sh
```

## 실행 방법

### 1. 하드웨어 안전 확인

실제 로봇이 움직이므로 처음에는 바퀴를 띄우거나 넓은 공간에서 시험한다.

- 모터 전원과 비상 정지 수단 확인
- 로봇 주변 최소 1 m 이상 확보
- `/dev/ttyCH341USB0`, `/dev/ttyCH341USB1` 존재 확인
- LiDAR가 회전하고 `/scan`이 들어오는지 확인
- 이전 navigation goal이 남아 있지 않은지 확인

Jetson에서:

```bash
ls -l /dev/ttyCH341USB0 /dev/ttyCH341USB1
```

### 2. Jetson 접속

Windows PowerShell 또는 WSL에서:

```bash
ssh odyssey@192.168.0.18
```

SSH 별칭을 등록했다면:

```bash
ssh jetson
```

### 3. Jetson navigation 실행 상태 확인

먼저 같은 stack이 이미 실행 중인지 확인한다.

```bash
source ~/argos_project/scripts/argos_env.sh
ros2 node list
pgrep -af 'argos_.*launch|nav2|slam_toolbox'
```

`/controller_server`, `/planner_server`, `/bt_navigator`가 이미 있으면 새 Navigation을 중복 실행하지 않는다. 수동 실행의 기준 명령은 다음과 같다.

```bash
ros2 launch argos_bringup argos_navigation.launch.py \
  map:=$HOME/argos_project/maps/argos_lab.yaml
```

user systemd 서비스를 별도로 설치한 환경에서만 다음 명령을 사용한다.

```bash
systemctl --user status argos-navigation.service --no-pager
journalctl --user -u argos-navigation.service -n 100 --no-pager
```

`argos_navigation.launch.py` 하나가 bringup, 고정맵, AMCL, planner, RPP, BT, velocity smoother를 모두 포함한다. 별도로 SLAM이나 localization launch를 중복 실행하지 않는다.

### 4. WSL에서 RViz 실행

WSL Ubuntu 24.04 / ROS 2 Jazzy 터미널에서:

```bash
bash "/mnt/c/Users/박현성/OneDrive/Documents/ChatGPT/argos/scripts/start_wsl_rviz.sh"
```

RViz 설정:

- Fixed Frame: `map`
- Map: `/map`
- LaserScan: `/scan`
- Odometry: `/odom`
- AMCL Particle Cloud: `/particle_cloud`
- Global Path: `/plan`
- Goal tool: Nav2 Goal

RViz의 `Navigation 2` 패널에서 **Pause**를 누르면 Jetson Nav2 lifecycle 노드가 비활성화되어 goal을 보내도 주행하지 않는다. 시험 중에는 Pause를 누르지 않는다.

### 5. 초기 위치 지정

1. 고정맵과 실제 로봇 위치가 일치하는지 확인한다.
2. 일치하지 않으면 RViz의 **2D Pose Estimate**를 선택한다.
3. 맵에서 로봇의 실제 위치를 클릭하고 실제 전방 방향으로 화살표를 드래그한다.
4. particle cloud와 LaserScan이 맵 벽에 정렬되는지 기다린다.
5. 정렬되지 않은 상태에서는 goal을 보내지 않는다.

### 6. Goal 전송

1. RViz에서 **Nav2 Goal**을 선택한다.
2. 도착 위치를 클릭한다.
3. 도착 시 로봇이 바라볼 방향으로 드래그한다.
4. `/plan`이 생성되는지 확인한다.
5. 로봇이 먼저 경로 방향으로 회전한 뒤 RPP 경로를 추종하는지 확인한다.

## 정상 여부 확인 명령

아래 명령은 Jetson에서 ROS 환경을 source한 뒤 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/odyssey/ydlidar_ws/install/setup.bash
source /home/odyssey/argos_project/ros2_ws/install/setup.bash
```

Lifecycle:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

세 노드 모두 `active [3]`이어야 한다.

활성 컨트롤러 확인:

```bash
ros2 param get /controller_server FollowPath.plugin
```

기대값:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

센서와 localization 확인:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

Goal/주행 중 확인:

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 action info /navigate_to_pose
```

로그 실시간 확인:

```bash
journalctl --user -fu argos-navigation.service
```

## Goal이 들어왔는데 움직이지 않을 때

다음 순서로 확인한다.

1. `systemctl --user is-active argos-navigation.service`가 `active`인지 확인
2. controller/planner/BT lifecycle이 모두 `active [3]`인지 확인
3. RViz `Navigation 2` 패널이 Pause 상태가 아닌지 확인
4. `/navigate_to_pose` action server가 보이는지 확인
5. goal 직후 `/plan`이 한 번 이상 발행되는지 확인
6. `/cmd_vel_nav`가 나오지만 `/cmd_vel`이 없으면 velocity smoother 확인
7. `/cmd_vel`이 있지만 바퀴가 안 돌면 base driver, serial port, watchdog 확인
8. `/plan`이 없으면 planner/costmap/TF 로그 확인
9. 경로가 있는데 controller가 실패하면 local costmap, robot pose, RPP 로그 확인

로봇이 갑자기 멈추면 무조건 controller 문제로 단정하지 않는다. BT cancel, lifecycle Pause, progress checker, TF 오류, collision check, velocity smoother, base-driver watchdog을 각각 구분해야 한다.

## 현재 알려진 문제점

### 1. LiDAR 회전수 불안정

YDLIDAR가 약 3.67 Hz와 11.76 Hz 사이에서 변동한 기록이 있다. 복원 직후에는 약 3.66 Hz였다. 느린 구간에서는 한 scan 동안 로봇이 크게 움직여 벽이 두꺼워지고 localization 및 costmap 시점 오차가 커질 수 있다.

가능한 원인과 확인 항목:

- 5 V 전원 전압 강하 또는 전류 부족
- 케이블/커넥터 접촉 문제
- LiDAR 모터 자체 불안정
- SDK 설정과 실제 장치 동작 불일치

### 2. LiDAR UART 체크섬 오류

128000 baud에서 간헐적인 checksum 오류가 확인되었다. 다만 checksum 로그가 존재한다는 사실만으로 경로 재계획의 직접 원인이라고 결론 내릴 수는 없다. `/scan` 손실률, AMCL pose 변화, TF 실패, costmap path invalid 시점을 같은 타임라인으로 기록해 인과관계를 확인해야 한다.

### 3. BT에 의한 주기적 경로 재계획

현재 BT는 경로가 유효해도 10초가 지나면 재계획한다. 따라서 같은 goal에서도 `/plan` 모양이 바뀔 수 있다. 기존 테스트에서 반복적인 경로 변화의 주된 직접 트리거는 컨트롤러가 아니라 이 BT 재계획 정책이었다.

확인해야 할 사항:

- 주기 재계획이 실제 장애물 대응에 필요한지
- `PathExpiringTimer`를 10초보다 길게 할지
- 시간 기반 재계획을 없애고 `IsPathValid`/goal 변경만 사용할지
- 재계획 전후 경로 길이·곡률·추종오차가 실제로 개선되는지

### 4. State Lattice 전역 경로 효율

전역 경로가 불필요하게 꺾이거나 회전 primitive를 과도하게 사용하는 현상이 남아 있다면 다음이 후보이다.

- 16개 heading의 낮은 각도 해상도
- `rotation_penalty`, `change_penalty`, `non_straight_penalty`
- inflation radius/cost scaling
- lattice motion primitive 형상
- 목표 방향 지정값
- 시작 AMCL 자세 오차
- smoothing이 lattice의 운동학적 의도를 흐리는지 여부

이 문제는 RPP와 분리해서 planner 출력 `/plan`만으로 평가해야 한다.

### 5. Wheel odometry와 skid slip

- IMU가 없다.
- 모터 제어가 closed-loop wheel velocity PID가 아니라 실측 선형 PWM 모델 기반 open-loop이다.
- skid-steer 특성상 제자리 회전과 바닥 마찰 변화에서 슬립이 커질 수 있다.
- AMCL이 장기 drift를 보정하지만 순간적인 pose 보정은 RPP에서 경로가 움직이는 것처럼 보이게 할 수 있다.

### 6. 시작 직후 TF 경고

기동 초기 `odom` 또는 `map` frame이 아직 준비되지 않아 transform timeout/message filter 경고가 몇 차례 나올 수 있다. 복원 검증에서는 수 초 뒤 모든 lifecycle 노드가 정상 활성화됐다. 경고가 계속되면 정상적인 startup transient가 아니므로 LiDAR timestamp, AMCL, odometry clock을 확인해야 한다.

### 7. ROS discovery 설정 불일치 가능성

프로젝트 기준값은 `ROS_DOMAIN_ID=42`이다. Jetson의 모든 실행 터미널과 원격 RViz 터미널에서 `source ~/argos_project/scripts/argos_env.sh`를 사용한다. 한쪽이 0이고 다른 쪽이 42이면 노드와 토픽이 전혀 보이지 않는다.

### 8. systemd 서비스가 transient unit

예전에 만든 `argos-navigation.service`가 transient unit이면 재부팅 뒤 사라질 수 있다. README의 수동 launch를 기준으로 사용하고, 자동 시작이 필요할 때 영구 user service 파일로 전환한다.

### 9. Collision Monitor 미사용

현재 `collision_monitor`는 구성에서 제거되어 있다. RPP의 자체 collision detection과 costmap은 사용하지만, 독립적인 최종 속도 안전 필터는 없다. 실물 주행 확대 전에 별도 안전 계층 도입을 검토해야 한다.

### 10. 실측되지 않은 값

- `base_link`가 차체 앞뒤 중앙이라는 가정
- velocity smoother의 가속/감속 한계
- 바닥 재질별 skid slip
- 실제 최소 회전 공간

## 앞으로 해결해야 할 과제

### P0 — RPP 기준 실주행 재검증

목표: 현재 baseline이 하나의 goal을 처음부터 끝까지 안정적으로 수행하는지 확인한다.

시험 시 동시에 기록할 것:

- `/plan`
- `/odom`
- `/amcl_pose`
- `/cmd_vel_nav`
- `/cmd_vel`
- `/scan` 주기
- Nav2 journal

완료 기준:

- goal action이 `SUCCEEDED`로 끝남
- 중간 lifecycle deactivation/cancel 없음
- controller failure 없음
- 계획 경로와 실제 pose의 횡방향 오차를 수치로 산출
- 재계획 횟수와 원인을 로그로 구분

### P0 — LiDAR 전원/UART 안정화

목표: scan frequency와 packet 품질을 안정화한다.

완료 기준:

- 장시간 scan frequency 분포 기록
- checksum 오류율 정량화
- 전원 전압을 정지/주행/회전 조건에서 측정
- 문제가 발생한 timestamp를 AMCL/TF/BT 로그와 대조

### P1 — BT 재계획 정책 A/B 테스트

다음 세 가지를 동일한 출발점·goal로 비교한다.

1. 현재 smart BT: 10초 + goal 변경 + invalid path
2. 시간 재계획 간격을 길게 한 smart BT
3. 진단용 one-shot BT: 최초 경로 한 번만 계산

비교 지표:

- goal 성공률
- 총 주행 시간과 거리
- 재계획 횟수
- 전역 경로 길이 변화
- 최대/평균 cross-track error
- recovery 횟수

### P1 — State Lattice 경로 품질 개선

RPP를 고정한 상태로 planner만 한 변수씩 조정한다.

우선순위:

1. 시작/goal pose와 AMCL 정렬 검증
2. 경로 길이·회전 횟수·최대 곡률을 자동 측정
3. penalty 튜닝
4. lattice heading/motion primitive 비교
5. smoother 적용 전후 비교

컨트롤러 변경으로 전역 경로 문제를 해결하려 하지 않는다.

### P1 — Odometry와 모터 제어 개선

- 직진 및 회전 calibration 재현성 확인
- 여러 바닥에서 effective track width 검증
- 좌우 wheel velocity closed-loop PID 검토
- 가능하면 IMU 융합 검토
- AMCL 보정 전후 odometry drift 정량화

### P2 — 안전 계층 추가

- Nav2 Collision Monitor 도입 검토
- 물리 비상 정지 장치 확인
- 센서 stale/TF stale 시 속도 차단
- recovery backup 속도와 거리 재검증

### P2 — 실행 환경 고정

- `ROS_DOMAIN_ID`를 Jetson/WSL 모두 한 값으로 통일
- WSL IP 변경을 고려한 DDS discovery 설정 정리
- transient unit을 영구 user systemd service로 변경
- 부팅 후 장치명, LiDAR, Nav2 자동 점검 추가

### P3 — MPPI 재검토 조건

다음 조건이 충족된 뒤에만 MPPI를 다시 비교한다.

- RPP baseline 주행 성공
- LiDAR/TF/odometry가 안정적
- State Lattice 전역 경로 품질이 별도로 검증됨
- 동적 장애물을 피해 원래 경로에서 벗어나는 기능이 실제 요구사항으로 확인됨

MPPI 비교 시 State Lattice의 path orientation을 critic이 사용하도록 설정하고, RPP와 동일한 속도·costmap·goal에서 정량 비교해야 한다.

## 변경 원칙

문제 원인을 분리하기 위해 다음 원칙을 지킨다.

1. 고정맵 + AMCL + State Lattice + BT + RPP 구조를 baseline으로 유지한다.
2. 지도 작성용 SLAM과 navigation을 동시에 실행하지 않는다.
3. planner 문제와 controller 문제를 구분한다.
4. 한 번의 시험에서는 한 종류의 파라미터만 변경한다.
5. 변경 전 설정을 백업하고 실험 목적을 기록한다.
6. 느낌이 아니라 timestamp가 있는 topic과 journal로 판단한다.
7. goal 성공 여부뿐 아니라 경로 길이, 추종 오차, 재계획 횟수, recovery를 함께 기록한다.

## 복원 기준 및 백업

2026-08-24 복원 기준:

- Git commit: `28c0b22` (`Add smart conditional replanning behavior tree`)
- 활성 BT: `config/navigate_to_pose_smart.xml`
- 활성 controller: RPP
- 활성 launch: `argos_navigation.launch.py`
- MPPI 직전 RPP 백업: `config/nav2_params.yaml.bak_rpp_before_mppi`
- 복원 시점 MPPI 설정 백업: `config/nav2_params.yaml.bak_mppi_20260824`

복원 직후 확인 결과:

- `controller_server`: active
- `planner_server`: active
- `bt_navigator`: active
- RPP plugin 로딩 확인
- State Lattice plugin 로딩 확인
- LiDAR `/scan`: 약 3.66 Hz

실물 goal 주행 결과는 별도의 테스트 로그와 함께 이 문서에 계속 추가한다.
