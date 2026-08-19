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
