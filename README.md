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

`telegram_mlp_spark_weight: 0.10`이어도 내부 YOLO spark confidence와
텔레그램 메시지의 spark 탐지값은 그대로 남는다. 오직 MLP 경보확률을 계산할
때만 spark 값을 10%로 줄인다. 예를 들어 화면의 `0.20`은 MLP에 `0.02`로
입력된다. 나중에 실제 spark 양성·음성 데이터를 포함해 MLP를 다시 학습한
뒤에는 이 값을 `1.0`으로 바꿀 수 있다.

카메라 HUD의 spark 박스와 방위 표시는 `spark_display_conf: 0.60` 이상일
때만 나타난다. 이 표시 기준은 내부 감지값이나 MLP 입력에는 영향을 주지 않는다.

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
map
└─ odom                 AMCL (고정맵) 또는 slam_toolbox (매핑)
   └─ base_link         EKF (융합 모드) 또는 wheel_odometry_node (fallback)
      ├─ laser_frame    static TF (실측)
      └─ imu_link       static TF (MPU6050)
```

TF 발행 주체를 중복 실행하면 안 된다.

- `map -> odom`: AMCL 또는 slam_toolbox 중 **하나만** 발행
- `odom -> base_link`: **정확히 하나만** 발행한다
  - 융합 모드(`use_ekf:=true`, 기본): `ekf_filter_node`가 발행하고
    `wheel_odometry_node`는 `publish_tf:=false`로 발행하지 않는다
  - fallback 모드(`use_ekf:=false`): `wheel_odometry_node`가 발행한다
- `base_link -> laser_frame`: `argos_bringup.launch.py`의 static transform publisher
- `base_link -> imu_link`: 같은 launch의 static transform publisher (`use_imu:=true`)
- `laser_frame -> imu_link` 직접 TF는 **만들지 않는다**. 둘의 상대 변환은
  TF tree가 `base_link`를 거쳐 간접 계산한다. LiDAR-IMU extrinsic
  calibration은 하지 않는다.
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
| `/odom` | 최종 odometry | EKF (융합) / wheel_odometry_node (fallback) |
| `/wheel/odom_raw` | 바퀴 엔코더 원시 odometry (융합 모드에서만) | wheel_odometry_node |
| `/imu/data_raw` | MPU6050 IMU 100 Hz | mpu6050_node |
| `/diagnostics` | IMU/EKF 상태 | mpu6050_node, ekf_filter_node |
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

## IMU (MPU6050) 융합

### 하드웨어 연결

| 항목 | 값 |
|---|---|
| 버스 | `/dev/i2c-1` (Jetson 40핀 헤더 **27번 SDA / 28번 SCL**) |
| 주소 | `0x68` (AD0 = GND) |
| `WHO_AM_I` | **`0x72`** |
| 전원 | 3.3V |

`WHO_AM_I`가 `0x72`이므로 **정품 InvenSense MPU-6050이 아니다.** 정품은 `0x68`을
반환한다. `0x72`는 GY-521 보드에 흔히 올라가는 **MPU-6500 계열 호환 다이**다.

온도 레지스터가 이를 확증한다. raw = 2099일 때

```text
MPU6050 식  2099/340    + 36.53 = 42.7 C   실온과 안 맞음
MPU6500 식  2099/333.87 + 21    = 27.3 C   MLX90640 실측 27 C 와 일치
```

자이로/가속도 레지스터 맵과 감도(FS_SEL=0 → 131 LSB/(deg/s),
AFS_SEL=0 → 16384 LSB/g)는 두 계열이 같으므로 스케일링은 분기하지 않는다.
다만 드라이버에서 **`WHO_AM_I == 0x68` 검사를 하면 안 된다.** 정상 장치를
거부하게 된다. `0x00` / `0xFF`(버스 죽음)만 걸러낸다.

`python3-smbus2`는 설치하지 않았다. `/dev/i2c-1`을 직접 열고
`ioctl(I2C_RDWR)`로 접근한다. `I2C_RDWR`을 쓰는 이유는 repeated-start
때문이다. `i2c-1`은 온보드 전원 모니터(INA3221 `0x40`, `0x25`)와 버스를
공유하므로, write와 read를 따로 하면 그 사이에 커널 드라이버가 끼어들어
엉뚱한 레지스터를 읽을 수 있다.

### 장착 축과 base_link -> imu_link

정지 상태 accel 평균(7초, 590 표본)이 장착 자세를 알려준다.

```text
accel x = -0.7311 m/s^2
accel y = +0.0963 m/s^2
accel z = +10.2347 m/s^2     <- 양수. 칩 윗면이 하늘을 향한다.
|a|     =  10.26  m/s^2      <- 중력 9.807 보다 4.6% 크다 (클론 스케일 오차)
기울기  = asin(-0.7311/10.26) = -4.09 deg
```

**IMU의 +Z가 base_link의 +Z와 같은 방향(둘 다 위)이다.** 따라서 z축 둘레의
회전 성분은 yaw를 얼마로 두든 그대로 보존된다. 보드가 앞/뒤/좌/우 어디를
보든 `angular_velocity.z`는 같다. 초기 융합에서 yaw 값이 정확하지 않아도
안전한 이유다.

설정 파일은 `config/imu_mount.yaml`이다. 현재 값은 전부 0이며, 위치(x/y/z)는
아직 자로 실측하지 않았다. **초기 EKF는 각속도만 쓰므로 위치 오차가 결과를
바꾸지 않는다.** 나중에 accel을 융합하게 되면 그때 실측해야 한다.

### 축 부호 검증 결과

```bash
python3 ~/argos_project/scripts/imu_axis_check.py
```

로봇을 손으로 돌려 측정한 결과(2026-08-26):

| 판정 항목 | 기준 | 실측 | 결과 |
|---|---|---|---|
| CCW(왼쪽) 회전 | `gz > 0` | **+37.27 deg/s** | 통과 |
| CW(오른쪽) 회전 | `gz < 0` | **-29.75 deg/s** | 통과 |
| 360도 적분 | 약 +360도 | **+355.4도** | 오차 -1.3% |

REP-103 규약과 부호가 일치한다. **코드에서 `-1`을 곱하는 보정이 필요 없다.**

만약 나중에 부호가 반대로 나오면, 값에 `-1`을 곱하지 말고
`config/imu_mount.yaml`의 `yaw`를 180도 돌려라. TF 회전으로 표현할 수 있는
것을 코드에서 부호로 때우면 나중에 accel을 융합할 때 반드시 어긋난다.

### gyro bias calibration

`mpu6050_node`는 기동할 때 `calibration_seconds`(기본 7초) 동안 gyro bias를
측정한다. **이 동안 로봇이 완전히 정지해 있어야 한다.**

2026-08-26 실측(7초, 590 표본, I2C 실패 0회):

| 축 | 평균 (rad/s) | 평균 (deg/s) | 표준편차 | 최대편차 |
|---|---|---|---|---|
| X | -0.031707 | -1.8167 | 0.004512 | 0.008928 |
| Y | +0.004557 | +0.2611 | 0.002442 | 0.006235 |
| **Z** | **-0.007303** | **-0.4184** | **0.001225** | 0.003489 |

측정한 z축 분산(`1.5e-06 (rad/s)^2`)은 IMU 메시지의
`angular_velocity_covariance`로 자동 승격된다. 실측이 파라미터 추정치보다 낫다.

bias를 빼지 않으면 z축이 **분당 25도**씩 돈다. 반드시 빼야 한다.
고정값을 쓰고 싶으면 `config/mpu6050.yaml`의 `gyro_bias_z`를 설정한다.

### MPU6050의 장기 yaw drift 한계

**MPU6050에는 magnetometer가 없다. 절대 yaw 기준이 없으므로 자체 적분한
yaw는 반드시 드리프트한다.** 이것은 고칠 수 없는 원리적 한계다.

드라이버는 이를 명시하기 위해 orientation을 아예 제공하지 않는다.

```text
orientation_covariance[0] = -1.0     (REP-145: "제공하지 않음")
```

bias 보정 후 실측 drift(정지 45초):

```text
yaw drift = -0.49 deg / 45초 = -0.66 deg/분
```

원시 bias가 -25 deg/분이었으므로 약 38배 개선됐지만 **0이 되지는 않는다.**
남은 것은 bias 불안정성(온도 드리프트 등)이다.

주의할 점이 하나 더 있다. **정지 상태에서는 wheel-only가 EKF보다 낫다.**
바퀴가 안 돌면 wheel odometry의 drift는 정확히 0인데, gyro는 계속 흘러간다.

```text
정지 45초 yaw drift    EKF        -0.49 deg
                       wheel-only  0.00 deg
```

반대로 **회전 중에는 gyro가 압도적으로 낫다.** skid-steer의 제자리 회전
슬립이 wheel odometry yaw를 크게 틀어놓기 때문이고, 그것이 이 융합의 목적이다.

장기적으로는 `map -> odom`을 내는 AMCL 또는 slam_toolbox가 절대 yaw를
보정하므로 이 드리프트는 누적되지 않는다. 하지만 **localization 없이
odometry만 쓰는 구간이 길어지면 반드시 문제가 된다.**

### wheel-only와 EKF 전환

기본값은 융합 모드다.

```bash
# 융합 모드 (기본) - wheel encoder + gyro z 를 EKF 로 융합
ros2 launch argos_bringup argos_slam.launch.py
```

```bash
# fallback 모드 - 기존과 완전히 동일한 wheel-only 동작
ros2 launch argos_bringup argos_slam.launch.py use_ekf:=false
```

```bash
# IMU 노드 자체를 끈다 (EKF 도 자동으로 꺼진다)
ros2 launch argos_bringup argos_slam.launch.py use_imu:=false
```

`use_imu` / `use_ekf` argument는 다음 4개 launch 파일 모두에서 쓸 수 있다.

- `argos_bringup.launch.py`
- `argos_slam.launch.py`
- `argos_navigation.launch.py`
- `argos_fire_patrol.launch.py`

두 모드의 데이터 흐름은 이렇다.

```text
융합 모드 (use_imu:=true use_ekf:=true, 기본)

    wheel encoder ─> /wheel/odom_raw ─┐
                                      ├─> EKF ─> /odom + odom->base_link
    MPU6050 ──────> /imu/data_raw ────┘

fallback 모드 (use_ekf:=false)

    wheel encoder ─> /odom + odom->base_link
```

**`use_ekf:=true`인데 `use_imu:=false`면 어떻게 되는가?**
EKF에 yaw rate를 줄 센서가 없어진다. `odom0`은 `vx`만 융합하므로 yaw가
영원히 갱신되지 않아 odometry가 완전히 망가진다. 그래서 두 argument를
AND로 묶어, **IMU가 없으면 EKF도 자동으로 끄고 fallback으로 내려간다.**
조용히 깨지는 것보다 낫다.

### EKF가 무엇을 융합하고 무엇을 융합하지 않는가

설정 파일은 `config/ekf_imu.yaml`이다.

| 입력 | 융합하는 것 | 융합하지 않는 것 |
|---|---|---|
| `/wheel/odom_raw` | `linear_velocity.x` | pose(x/y/yaw), `vy`, `vyaw` |
| `/imu/data_raw` | `angular_velocity.z` | orientation, linear acceleration |

각각의 이유는 이렇다.

- **wheel pose를 안 쓰는 이유**: 이미 적분된 값이라 슬립 오차가 누적돼 있다.
  융합하면 EKF가 wheel의 누적 yaw 오차를 그대로 물려받아 gyro를 넣은 의미가
  사라진다.
- **wheel `vyaw`를 안 쓰는 이유**: 이번 작업의 핵심이다. wheel의 yaw rate는
  제자리 회전에서 슬립으로 가장 크게 틀어지는 양이고, 정확히 그것을 gyro로
  대체하려는 것이다. 둘을 같이 높은 신뢰도로 넣으면 EKF가 평균을 내면서
  슬립 오차가 절반만 줄어든다.
- **wheel `vy`를 안 쓰는 이유**: 비홀로노믹 제약으로 `vy = 0`을 넣고 싶을 수
  있다. 하지만 skid-steer는 회전할 때 좌우 바퀴가 실제로 옆으로 미끄러진다.
  차체 중심에서 본 순간 속도에는 0이 아닌 y 성분이 존재한다. 여기에 "vy는
  정확히 0"이라는 강한 제약을 걸면 필터가 회전 중에 그 모순을 x/yaw로
  떠넘겨 오히려 왜곡된다. `two_d_mode: true`가 z/roll/pitch를 이미 묶어준다.
- **IMU orientation을 안 쓰는 이유**: magnetometer가 없어 절대 yaw 기준이 없다.
- **IMU linear acceleration을 안 쓰는 이유**: 이 클론 다이는 `|a|`가
  중력보다 4.6% 크다. 스케일 오차가 있는 가속도를 두 번 적분하면 위치가
  급격히 발산한다. 바퀴 엔코더가 이미 훨씬 좋은 선속도를 준다.

`wheel_odometry_node`의 covariance도 이번에 채웠다. 기존에는 전부 0이었는데,
혼자 쓸 때는 아무도 안 보지만 EKF에 넣으면 robot_localization이
**"분산 0 = 무한 신뢰"로 해석해서 필터가 발산하거나 IMU를 완전히 무시한다.**

### 진단 명령

```bash
source ~/argos_project/scripts/argos_env.sh
```

IMU 발행 상태

```bash
ros2 topic hz /imu/data_raw
```

```bash
ros2 topic echo /imu/data_raw --once
```

IMU / EKF 진단 (I2C 오류 횟수, bias, jitter, 필터 상태)

```bash
ros2 topic echo /diagnostics
```

발행자가 정확히 하나인지 확인 (가장 중요한 점검)

```bash
ros2 topic info /odom -v
```

```bash
ros2 run tf2_ros tf2_monitor odom base_link
```

TF 트리 연결 확인

```bash
ros2 run tf2_ros tf2_echo map laser_frame
```

축 부호와 적분 오차 검증

```bash
python3 ~/argos_project/scripts/imu_axis_check.py --with-odom
```

정지 상태 drift 측정 (wheel-only와 EKF 비교용)

```bash
python3 ~/argos_project/scripts/odom_drift_check.py --seconds 45
```

LiDAR 주기 / timestamp 진단 (IMU와 별개 문제)

```bash
python3 ~/argos_project/scripts/scan_diagnose.py --seconds 60
```

I2C 장치 확인 (버스 전체를 훑지 말고 주소를 지정할 것)

```bash
i2cget -y 1 0x68 0x75 b
```

### 매핑 명령

기존 지도는 절대 덮어쓰지 않는다. 새 이름을 쓴다.

```bash
ros2 launch argos_bringup argos_slam.launch.py
```

주행은 천천히 한다.

- 속도 약 `0.05 ~ 0.08 m/s`
- 급가속 금지, 빠른 제자리 회전 금지
- 같은 벽을 최소 두 번 겹쳐 주행
- 시작 위치로 돌아와 loop closure 생성
- 벽과 너무 가까이 붙지 말 것
- 긴 복도에서는 특징이 있는 구역을 함께 통과할 것

저장한다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/argos_project/maps/argos_lab_imu_v1
```

wheel-only 지도와 비교하려면 같은 경로를 `use_ekf:=false`로 한 번 더 돈다.

```bash
ros2 launch argos_bringup argos_slam.launch.py use_ekf:=false
```

### LiDAR 문제는 IMU로 해결되지 않는다

**IMU를 넣어도 LiDAR 자체 문제는 그대로 남는다.** 둘은 별개다.

`scan_diagnose.py` 실측(2026-08-26, 60초):

| 항목 | `/scan_raw` | `/scan` |
|---|---|---|
| 주기 | 11.678 Hz | 11.678 Hz |
| 주기 표준편차 | 10.1% | 13.5% |
| 끊김(2배 초과) | 1회 (0.275초) | 1회 (0.350초) |
| timestamp 역행 | **0회** | **0회** |
| `scan_time` vs 실제 | 0.08553 vs 0.0856 (일치) | 일치 |
| 점 개수 | **427** | 1440 |
| 유효 측정점 | 91.5% | **27.2%** |

**LiDAR timestamp 자체는 건강하다.** 역행이 없고, `scan_time`과
`time_increment`가 실제 주기와 일치하며, 60초간 stamp 경과와 수신 경과의
차이가 +0.001초다.

**그런데 `scan_normalizer`의 `num_bins: 1440`이 현재 조건과 안 맞는다.**

`/scan_raw`가 스캔당 427점인데 1440 bin에 재샘플링한다.
`427/1440 = 최대 29.7%`만 채워지고, 실측 유효율 27.2%가 이와 일치한다.
**1440개 중 약 1050개가 빈 bin이다.**

원인은 회전 속도다. 센서의 샘플링 레이트는 약 5 kHz로 고정이고 회전 속도만 변한다.

```text
현재       427점 x 11.678 Hz = 4,987 점/초
코드 주석 1351점 x  3.67 Hz = 4,958 점/초    <- 같다
```

`num_bins: 1440`은 3.67 Hz(1351점) 기준으로 튜닝된 값인데, 지금은 11.678 Hz로
돌아 427점만 들어온다. 실제 각도 분해능은 `360/427 = 0.84도`인데 0.25도
격자에 넣으므로, 각 점이 어느 bin에 떨어지는지가 회전 위상에 따라 스캔마다
달라진다. **벽이 뭉개지고 두꺼워지는 원인이며 IMU와 무관하다.**

**2026-08-26 수정: `num_bins`를 1440에서 450으로 낮췄다.**
`argos_bringup.launch.py`의 `scan_bins` argument로 조절한다.

낮춰야 했던 진짜 이유는 빈 bin 자체가 아니라 **CPU 부하**였다.

```text
예전  3.67 Hz x 1440 =  5,285 점/초
전    11.678 Hz x 1440 = 16,816 점/초   <- 3.2 배
후    11.678 Hz x  450 =  5,255 점/초   <- 예전 수준 복귀
```

Karto는 스캔 하나를 처리할 때 `LocalizedRangeScan::Update()`와
`OccupancyGrid::AddScan()`에서 **전체 reading 수만큼** 순회한다. 즉 비용이
유효 점 개수가 아니라 `num_bins`에 비례한다. 유효 점이 302개뿐인데 1440을
돌고 있었으니 4.8배를 낭비한 셈이다.

그 결과 slam_toolbox가 입력을 따라가지 못하고 스캔을 버렸다.

```text
[slam_toolbox]: Message Filter dropping message: frame 'laser_frame'
                reason 'discarding message because the queue is full'
```

실측 효과 (같은 환경, 45초 관찰):

| 항목 | 1440 | 450 |
|---|---|---|
| bin 충전율 | 25 % | **70 %** |
| scan drop | 61 회 | **0 회** |
| Karto 부하 | 16,816 점/초 | 5,255 점/초 |

**빈 bin이 벽을 지운다는 가설은 틀렸다.** `Karto.h`의 `OccupancyGrid::AddScan`을
직접 확인한 결과, `rangeReading <= minRange`인 값은 raytracing 전에 걸러진다.

```cpp
if (rangeReading <= minRange || rangeReading >= maxRange || std::isnan(rangeReading)) {
    // ignore these readings
    continue;
}
```

`range_min`이 0.1이고 빈 bin이 0.0이므로 그냥 무시된다. 지도를 훼손하지 않는다.

LiDAR 회전수가 다시 내려가면 `num_bins`를 실측해서 올려야 한다.
`scripts/scan_diagnose.py`로 회전당 점 개수를 먼저 잰다.

### stamp_at_midpoint 검증

`scan_normalizer`의 `stamp_at_midpoint: true`가 옳은지 확인했다.

- LaserScan 규약상 `header.stamp`는 **첫 번째 광선**의 시각이다.
  YDLidar SDK도 `outscan.stamp = global_nodes[0].stamp`로 그렇게 넣는다.
- 그런데 `slam_toolbox`(Karto)와 Nav2 costmap은 **광선별 motion
  compensation을 하지 않는다.** 스캔 전체를 `header.stamp` 한 시각의
  자세로 정합한다.
- 한 바퀴가 0.0856초이므로, 첫 광선 시각을 쓰면 평균 `scan_time/2 = 0.043초`
  뒤처진 자세를 쓰게 된다. 0.6 rad/s로 회전 중이면 **약 1.5도의 계통 오차**이고,
  회전 방향이 바뀔 때마다 반대로 틀어져 벽이 이중으로 찍힌다.

**결론: 이 스택에서는 `stamp_at_midpoint: true`가 맞다.** 규약을 엄격히
따르는 것보다 실제 소비자(slam_toolbox, Nav2)의 동작에 맞추는 편이 낫다.

**단, 주의할 것이 있다.** 이 설정은 LaserScan 규약을 위반하므로, 나중에
광선별 dewarping을 하는 소비자(예: `laser_geometry`의 고정밀 모드,
일부 ICP 구현)를 붙이면 그쪽이 `time_increment`로 보정할 때
**반대 방향으로 반 스캔만큼 틀어진다.** 그런 노드를 추가할 때는
이 파라미터를 다시 검토해야 한다.

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

- ~~IMU가 없다.~~ 2026-08-26 MPU6050(MPU-6500 호환 클론)을 추가하고
  `robot_localization` EKF로 gyro z를 융합했다. 위 "IMU (MPU6050) 융합" 절 참고.
  단 magnetometer가 없어 장기 yaw drift(약 -0.66 deg/분)는 남는다.
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
- ~~가능하면 IMU 융합 검토~~ 2026-08-26 완료 (MPU6050 + robot_localization EKF)
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
