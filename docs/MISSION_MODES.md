# ARGOS 사용자 선택 모드

이 통합은 인지팀의 `scripts/new_main_robot_map.py`를 수정하거나 import하지
않는다. 메인봇의 기존 화재 인지/Nav 기능과 팔로우봇의 기존 ArUco 카메라를
그대로 두고, 팔로우봇의 단일 감독기가 모터와 펌프 명령을 중재한다.

## 모드

| 모드 | ArUco/궤적 추종 | 좌표 화재 지령 | 용도 |
|---|---:|---:|---|
| `auto` | 허용 | 허용 | 기존 동작을 보존하는 기본값 |
| `follow` | 허용 | 거부 | 사용자를 따라가는 모드 |
| `coordinate_fire` | 정지 | 허용 | 지정된 `map` 좌표로 이동·살수·복귀 |
| `standby` | 정지 | 거부 | 대기/정비 |

모드가 실제로 바뀌면 감독기는 진행 중 Nav2 목표를 취소하고, 현재 화재
미션을 취소·초기화하며, 모터 `S`와 펌프 `P,0`을 먼저 보낸다. 같은 모드를
다시 요청하는 것은 상태 확인으로 처리하므로 준비 중인 좌표 미션이 지워지지
않는다.

## 데이터 흐름

```text
팔로우봇 카메라 -> ArUco(ID 5) -> follow_controller --결정만--> fire_supervisor
                                                               |
사용자 argos_mode.py -> /follower/mode/set --------------------+
사용자/메인봇       -> /fire/dispatch -------------------------+
                                                               v
                    단일 명령 소유자 -> motor_command / pump_command
```

추종 제어기의 `publish_commands`는 항상 `false`이다. 실제 명령은
`fire_supervisor`만 발행한다. 메인봇 YOLO와 팔로우봇 ArUco는 서로 다른
로봇·카메라에서 동작하므로 YOLO나 메인 카메라가 중복 생성되지 않는다.

## 사용자 명령

메인봇에서 ROS 환경과 이 저장소 환경을 source한 뒤 실행한다.

```bash
python3 scripts/argos_mode.py status
python3 scripts/argos_mode.py set follow
python3 scripts/argos_mode.py set standby
python3 scripts/argos_mode.py set auto
```

좌표 화재 임무는 먼저 이동 금지 상태로 준비할 수 있다.

```bash
python3 scripts/argos_mode.py coordinate-fire \
  --x 1.20 --y -0.40 --yaw-deg 90 --mission-id fire-demo-1
```

출력에서 `WAIT_CLEARANCE`를 확인하고 메인봇과 사람이 경로에서 비킨 후,
같은 좌표와 같은 mission ID로 명시적으로 이동을 허용한다.

```bash
python3 scripts/argos_mode.py coordinate-fire \
  --x 1.20 --y -0.40 --yaw-deg 90 --mission-id fire-demo-1 \
  --confirm-main-clear
```

취소와 초기화:

```bash
python3 scripts/argos_mode.py cancel --mission-id fire-demo-1
python3 scripts/argos_mode.py reset
```

`argos_mode.py`는 모터나 펌프를 활성화할 수 없다. `enable_motion`과
`enable_pump`는 팔로우봇의 로컬 launch 설정만이 결정한다. 펌프가 꺼진
dry-run에서도 이동/도착 상태 기계는 시험할 수 있고 실제 살수는 하지 않는다.

전체 기동은 기본적으로 `standby`, 모터 꺼짐, 펌프 꺼짐이다. 단계별 opt-in은
팔로우봇에서 다음처럼 한다.

```bash
# 통신/상태만: 움직이지 않음
~/fire_test_logs/run_follower_bringup.sh

# 주행 시험: 펌프는 계속 꺼짐
ARGOS_INITIAL_MODE=standby ARGOS_ENABLE_MOTION=true \
  ARGOS_ENABLE_PUMP=false ~/fire_test_logs/run_follower_bringup.sh

# 최종 자동 진화: 안전 구역과 물 계통을 확인한 경우에만
ARGOS_INITIAL_MODE=auto ARGOS_ENABLE_MOTION=true \
  ARGOS_ENABLE_PUMP=true ~/fire_test_logs/run_follower_bringup.sh
```

## ROS 인터페이스

- 모드 요청: `/follower/mode/set` (`std_msgs/String`, strict JSON)
- 모드 상태: `/follower/mode/status` (`std_msgs/String`, transient-local JSON)
- 화재 좌표: `/fire/dispatch` (`std_msgs/String`, strict JSON)
- 화재 상태: `/follower/fire/status` (`std_msgs/String` JSON)
- 취소: `/fire/cancel`
- 초기화: `/fire/reset`

모드 요청 예시:

```json
{"schema":1,"request_id":"mode-123","mode":"follow"}
```

좌표는 반드시 `map` 프레임이고, 살수 시간과 펌프 허용 여부는 원격 지령이
바꿀 수 없다.

## 단계별 실제 장비 시험

1. 바퀴를 띄우거나 궤도를 바닥에서 분리하고 `enable_motion=false`,
   `enable_pump=false`로 토픽과 모드 상태만 확인한다.
2. 펌프 전원을 분리한 채 `enable_motion=true`로 `follow` 저속 시험을 한다.
3. 펌프를 계속 끈 채 짧은 좌표 미션의 이동·도착·복귀를 시험한다.
4. 물이 전자장치에 닿지 않는 장소에서만 로컬 확인 문구를 사용해 펌프를
   활성화한다.

## 원복

기능 브랜치 적용 전 상태는 GitHub `main`의 커밋 `d087814`이다.
운영 장비에서 새 코드를 적용할 때는 먼저 기존 파일을 타임스탬프 백업한 후
배포한다. 문제가 있으면 해당 백업을 복사해 되돌리거나 `d087814`의 파일을
다시 배포한다. `new_main_robot_map.py`는 이 작업에서 변경되지 않는다.
