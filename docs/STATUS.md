# ARGOS 진행 상태

최종 갱신: 2026-08-20

## 완료 (실제 하드웨어 검증 완료)

### Phase A/B - Base driver
- `argos_base_driver` : /cmd_vel -> 모터, 엔코더 -> /wheel_ticks/*
- 모터 명령 좌우 순서 확정: `M,a,b` 에서 **a = RIGHT, b = LEFT**
  (calibrate_rotation_only.py 가 M,+55,-55 를 보냈을 때
   LEFT -1.556 m / RIGHT +1.561 m = CCW 인 것으로 확정)
- 속도 <-> PWM 실측 선형 모델, R^2 = 0.99952
  `speed = 0.00139532 * pwm - 0.00932066`
  최대 바퀴 속도 0.158 m/s, 최대 제자리 회전 0.637 rad/s
- 검증: 명령 v=0.10 -> 실측 0.103, w=0.50 -> 실측 0.488 (오차 3% 이내)
- fail-safe: watchdog 0.5 s, SIGTERM/atexit 정지

### Phase C~F - SLAM
- slam_toolbox online async, base_frame = base_link
- 맵 저장: `maps/argos_lab.{pgm,yaml}`

### Phase G - AMCL localization
- `argos_localization.launch.py` (map_server + amcl + lifecycle_manager)
- 검증 (scripts/check_localization.py):
  1.2 m 왕복 2회 후 중앙값 0.000 m / 10 cm 이내 90%
  360 deg 제자리 회전 후 중앙값 0.050 m / 10 cm 이내 83%

## 해결한 주요 결함

1. `package.xml` 중복 depend -> colcon 매니페스트 파싱 실패 -> ament 미등록
2. 모터 명령 좌우 반전 (angular.z>0 이 CW 로 돌던 문제)
3. 시리얼 부분 라인 파싱 (tick 튐)
4. LiDAR 스캔 좌우 반전 -> ydlidar `inverted: true`
5. LiDAR TF 가 틀림 -> 실측 x=+0.118 m, yaw=176.8 deg (12 cm 앞, 뒤 방향 장착)
6. `fixed_resolution` 이 회전을 잘라내 360 deg 중 113 deg 만 담기던 문제
7. slam_toolbox 를 LifecycleNode 로 안 띄워 /scan 구독조차 안 되던 문제
8. 스캔 점 개수 변동 -> slam_toolbox 가 전량 거부 -> `scan_normalizer` 도입
9. 스캔 타임스탬프 편향 (시작 시각 -> 중앙 시각 보정)

## 알려진 하드웨어 문제

- **LiDAR 모터 회전수 불안정**: 3.67 Hz <-> 11.76 Hz 사이를 오감
  (샘플레이트는 양쪽 다 5 kHz 로 사양 정상. 모터 속도만 널뜀)
  - `/dev/ttyTHS1` 은 GPIO UART 라 DTR 로 모터 제어 불가 (시험 완료, 무효)
  - 5V 공급 전류/전압 강하 점검 필요
  - 3.67 Hz 일 때 0.25 rad/s 회전이면 스캔 1장 동안 3.9 deg 회전 ->
    3 m 거리 벽이 20 cm 번짐. 현재 맵 벽이 두꺼운 주된 이유.
- **LiDAR UART 체크섬 에러** 간헐 발생 (128000 baud)
- **CH341 커널 모듈이 언로드된 적 있음** -> `scripts/install_ch341_persistent.sh` 로 영구화 필요
- CH340 2개가 VID:PID 동일 + 시리얼 없음 -> 물리 포트 경로로만 구분 가능
  (`1-2.2` = 오른쪽/모터, `1-2.4` = 왼쪽)

## 다음 단계 (Phase H~J) - 로봇 실측 치수 대기 중

Nav2 costmap footprint 를 임의값으로 정하지 않기 위해 다음이 필요하다.

- 차체 전체 길이 (앞끝 ~ 뒤끝, 궤도 포함)
- 차체 전체 폭 (좌우 궤도 바깥면)
- 구동륜 축 중심의 앞뒤 위치

참고: LiDAR 는 구동륜 축 중심보다 11.8 cm 앞에 있는 것으로 실측됨.

이후 순서: footprint -> Nav2 costmap -> SmacPlannerLattice -> RPP -> MPPI

## 2026-08-25 추가

### 치명적 하드웨어 함정: CH340 포트 뒤바뀜

모터가 전혀 안 움직이는 증상이 발생했다. 원인은 두 UNO 의 포트가
설정과 반대로 물린 것이었다.

각 보드는 자기 접두사를 내보낸다.

    R=  right_motor_encoder.ino   오른쪽 엔코더 + 모터 제어
    L=  left_encoder_nav.ino      왼쪽 엔코더 전용

그런데 실제로는 `/dev/ttyCH341USB0` 이 `L=` 을, `USB1` 이 `R=` 을 냈다.
즉 모터 명령을 받는 포트에 **모터 코드가 없는 보드**가 물려 있었고,
그 보드는 `M,...` 을 그냥 무시한다. 시리얼도 살아있고 엔코더 토픽도
50 Hz 로 정상 발행되므로 로그만 봐서는 정상으로 보인다.

CH340 두 개는 VID:PID 가 같고 시리얼 번호도 없어서
USB0 / USB1 순서가 재부팅이나 재연결로 뒤바뀔 수 있다.

**대응**: `argos_base_driver` 에 `auto_detect_ports` 를 추가했다.
기동 시 두 포트에서 몇 줄 읽어 `L=` / `R=` 접두사로 역할을 판별하고,
설정과 반대면 자동으로 바꿔 쓰며 경고를 남긴다.

검증: `M,60,60` 2초에 R= 0→+2448, L= 0→-2317.
기존 캘리브레이션 부호(`left_sign: -1`, `right_sign: +1`)와 일치하여
좌 +0.170 m / 우 +0.159 m 로 둘 다 전진.

### fire_seeker 벽 추종 추가

`--patrol wall|bounce`, `--side right|left` 로 전환한다.

벽 추종은 빔 두 개(옆 90도, 대각 45도)로 벽까지 거리와 벽의 기울기를
함께 추정한다. 거리 하나만 P 제어하면 좌우로 진동한다.
수식은 합성 데이터로 검증했다 (기하 30/30, 제어 부호 4/4).

### fire_seeker 방위각 끊김 보정

MLP 는 95~99% 로 계속 "불" 이라고 하는데 YOLO bbox 는 프레임의
1/3 에서만 잡히는 상황이 있었다. 방위각이 없으면 ALIGN 이 멈추고
LOST_SECONDS 뒤 PATROL 로 튕겼다가 즉시 FIRE_STOP 으로 되돌아와서
APPROACH 진입이 0회였다.

`BEARING_HOLD_SECONDS = 1.2` 로 마지막 방위각을 짧게 유지하도록 했다.
결과: APPROACH 0회 -> 10회, HOLD 0회 -> 7회.

### 남은 확인 사항

### MLP 미완성 -> 당분간 --gate yolo 사용

MLP 는 아직 학습이 끝나지 않은 상태다. 불이 전혀 없는 환경에서도
0.958 을 내놓는다. 입력은 정상이었다.

    YOLO   fire 0.000  smoke 0.000  spark 0.000  cigarette 0.009
    센서   온도 34~36 C, 가스 101 (정상 실내값)
    MLP    0.958

pkl 자체는 StandardScaler + MLPClassifier 파이프라인이고
feature 이름/순서/클래스 인덱스는 코드와 일치한다.
즉 배선 문제가 아니라 모델이 아직 덜 학습된 것이다.

이 상태로 `--gate mlp` 를 쓰면 로봇이 계속 화재 상태에 머물러
순찰을 못 한다 (실측: PATROL 진입 2회, TURN/FIRE_STOP/ALIGN 만 반복).

MLP 학습이 끝날 때까지는 `--gate yolo` 를 쓴다.

    --gate yolo 실측 (75초)
      PATROL 15회 / TURN 16회 / 화재 상태 오진입 0회 / 이동 0.87 m
- 맵 기반 Nav2 순찰은 아직 미구현. `/cmd_vel` 중재기가 먼저 필요하다
  (twist_mux 미설치).

### 실화염 검증 완료 (2026-08-25)

라이터 불꽃으로 순찰 -> 접근 -> 정지 전체 사슬을 실제로 확인했다.

실행:
    fire_seeker.py --patrol wall --side right --gate yolo --fire-conf 0.2

결과 (90초):
    PATROL 7 / TURN 8 / FIRE_STOP 1 / ALIGN 3 / APPROACH 3 / HOLD 1
    이동 1.13 m

    PATROL -> FIRE_STOP  전방 1.10 m, 불 -3deg
           -> ALIGN      불 -5deg
           -> APPROACH   전방 1.07 m
           -> ALIGN      불 +21deg   (접근 중 재정렬)
           -> APPROACH   불 +7deg
           -> ALIGN      불 +23deg
           -> APPROACH   불 +7deg
           -> HOLD       전방 0.60 m, 불 +6deg

전방 거리 1.10 -> 0.60 m 로 단조 감소. FIRE_STOP_DIST 에 정확히 정지.

임계값
------
기본 YOLO_ONLY_FIRE_CONF = 0.40 은 라이터 불꽃에 너무 높다.
0.40 으로 돌렸을 때는 화재 상태 진입이 0회였다.
라이터 기준으로는 --fire-conf 0.2 가 동작한다.
불꽃 크기가 다르면 scripts/check_fire_detect.py 로 다시 재서 정할 것.

방위각 부호
-----------
불이 -3deg -> +6deg 로 수렴하며 실제로 불에 접근했으므로
CAMERA_YAW_OFFSET = 0.0 (카메라 정면) 이 맞다.
LiDAR 는 176.8deg 뒤집혀 있지만 카메라는 정면이다.

남은 거칠기
-----------
접근 중 ALIGN <-> APPROACH 를 3회 오갔다.
REALIGN_DEG(20deg)을 넘는 방위각 흔들림 때문이다.
동작에는 지장이 없었다. 더 부드럽게 하려면
REALIGN_DEG 를 키우거나 APPROACH_W_MAX 를 낮춘다.
