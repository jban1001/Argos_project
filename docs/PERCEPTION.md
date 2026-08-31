# ARGOS 화재 인지 (fire_perception_main)

인지팀 원본 `~/YOLO/new_main_robot_map.py` 를 **한 바이트도 고치지 않고**
그대로 실행시키면서, 그 결과를 ROS 2 토픽으로 중계하고 텔레그램 알림
동작을 바깥에서 조정하는 계층이다.

## 파일 경로

| 파일 | 역할 |
|---|---|
| `~/argos_project/scripts/fire_perception_main.py` | 이 문서가 설명하는 인지 래퍼 |
| `~/YOLO/new_main_robot_map.py` | 인지팀 원본. **수정 금지** |
| `~/YOLO/nabil_map.png` | 텔레그램용 건물 도면 |
| `~/YOLO/robot.png` | 도면에 얹는 로봇 아이콘 |
| `~/YOLO/fire_alerts/` | 알림 사진 임시 저장 (전송 성공 시 삭제) |
| `~/YOLO/best.engine` | YOLO TensorRT 엔진 |
| `~/YOLO/fire_mlp.pkl` | 위험도 MLP |
| `~/argos_project/scripts/fire_nav_integrated.py` | 주행 쪽. 이 파일과 토픽으로만 통신 |
| `~/argos_project/config/fire_nav_integrated.yaml` | 주행 쪽 파라미터 |

백업본은 `scripts/fire_perception_main.py.bak_<날짜>` 형식으로 남아 있다.

## 실행

```bash
source ~/argos_project/scripts/argos_env.sh && ~/.venv/bin/python ~/argos_project/scripts/fire_perception_main.py 2>&1 | grep --line-buffered -v "Corrupt JPEG data"
```

`~/.venv/bin/python` 이어야 한다. ultralytics 와 TensorRT 가 거기 있다.

**반드시 화면이 있는 터미널에서 실행한다.** 원본이 `cv2.imshow` 를 쓴다.
실측 결과 `DISPLAY` 가 아예 없으면 정상 동작하지만, 설정됐는데 권한이
없으면 Qt 가 abort 한다. SSH 에서 `DISPLAY` 를 억지로 export 하지 말 것.

## 왜 import 가 아니라 exec 인가

원본에는 `if __name__ == "__main__"` 가드가 없다. 601 줄부터 끝까지가 전부
모듈 최상위 실행문이라서 `import` 하면 그 자리에서 YOLO 로드 · 카메라 open ·
시리얼 open · `rclpy.init` · `while True` 를 전부 실행하고 **영원히
반환되지 않는다**. 그래서 별도 스레드에서 `exec` 한다.

`exec` 의 네임스페이스(`ns`)가 곧 원본의 모듈 전역이다. 원본 루프가 최상위에
있으므로 루프 안 변수가 전부 `ns` 에 그대로 보인다. 이게 이 통합의 유일한
hook 이며, 원본을 수정하지 않고 값을 읽고 함수를 바꿔 끼울 수 있는 이유다.

## 프레임 동기화

`cv2.imshow` 는 원본 루프의 마지막 문장이다. 이걸 우리 프로세스 안에서만
감싸서 "이번 프레임 계산 끝" 신호로 쓴다. 콜백이 원본 스레드 안에서 동기로
돌기 때문에 콜백이 도는 동안 원본은 그 자리에 멈춰 있다. 따라서 스냅샷에
서로 다른 프레임이 섞이지 않으며 락이 필요 없다.

`imshow` 가 끊기면 `poll_fallback_hz` 로 폴링 전환한다.

## 토픽

| 토픽 | 방향 | 형식 | 내용 |
|---|---|---|---|
| `/argos/fire_detection` | 발행 | `std_msgs/String` (JSON) | 매 프레임 인지 결과 |
| `/argos/fire_alert_request` | 구독 | `std_msgs/String` (JSON) | 주행 쪽이 HOLD 에서 보내는 알림 요청 |
| `/argos/fire_episode` | 구독 | `std_msgs/Bool` | 화재 처리 구간 시작/끝 |

### `/argos/fire_detection` 구조

```json
{
  "seq": 1234,
  "stamp": 1788110110.123,
  "frame": {"w": 1280, "h": 720},
  "confs": {"fire": 0.62, "smoke": 0.01, "cigarette_butt": 0.0, "spark": 0.0},
  "boxes": {
    "fire": {"conf": 0.62, "x1": 500, "y1": 300, "x2": 640, "y2": 460,
             "cx": 570.0, "cy": 380.0, "norm_x": -0.109}
  },
  "sensor": {"ok": true, "temp": 33.8, "gas": 38},
  "mlp_prob": 0.81,
  "danger_duration": 2.4,
  "telegram": {"state": "sent"},
  "mlp_danger": true,
  "alert_expected": true
}
```

`norm_x` 는 화면 중앙 기준 -1 ~ +1 의 순수 기하값이다. 카메라 화각을 모르는
값이라, 방위각 변환은 주행 쪽(`fire_nav_integrated.py`)이 담당한다.
인지팀이 화면 표시 기준(`DISPLAY_CONF`)을 어떻게 바꾸든 주행에 영향이 없다.

`telegram.state` 는 `idle` → `queued` → `sending` → `sent`(또는 `failed`).

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `detection_topic` | `/argos/fire_detection` | 인지 결과 발행 토픽 |
| `alert_request_topic` | `/argos/fire_alert_request` | 알림 요청 구독 토픽 |
| `map_yaml` | `~/argos_project/maps/argos_outdoor_imu_v1.yaml` | 정확 위치 지도용. **Nav2 가 쓰는 것과 같아야 한다** |
| `telegram_cooldown_seconds` | `90.0` | 원본 알림 최소 간격 (아래 설명) |
| `suppress_original_alert` | `false` | 원본 자체 알림을 완전히 차단 |
| `send_hold_alert` | `false` | HOLD 지점에서 우리가 별도 알림을 보낼지 |
| `show_window` | `true` | `cv2.imshow` 창 표시 |
| `tick_timeout` | `1.0` | imshow tick 이 끊겼다고 판단할 시간 [s] |
| `poll_fallback_hz` | `30.0` | tick 이 끊겼을 때 폴링 주기 |
| `log_period` | `5.0` | 상태 로그 주기 [s] |

## 원본에 거는 네 가지 개입

전부 `exec` 이후 `ns` 의 이름만 바꿔 끼우는 방식이다. 원본 파일은 그대로다.

### 1. 쿨다운 덮어쓰기 (`apply_overrides`)

원본 `L104 ALERT_COOLDOWN_SECONDS = 60` 을 `telegram_cooldown_seconds` 로
바꾼다. 원본 `L930` 이 매 루프마다 이 전역을 다시 읽으므로 즉시 반영된다.

**왜 90 초인가** — 원본은 조건이 유지되는 한 쿨다운마다 계속 보낸다.
그런데 불을 발견하고 접근을 마치고 순찰로 돌아가기까지가 최악 62 초다.

```
확정 1.0 + 정지 0.7 + 텔레그램대기 10.0 + 정렬 약 10 + 접근 25.0 + HOLD 15.0
```

그 동안 불은 계속 보이므로 조건도 계속 참이다. 20 초로 두면 한 화재 건에
알림이 4 회 간다. 접근 관련 파라미터를 늘리면 이 값도 같이 늘려야 한다.

### 2. 지도 개선 (`install_map_upgrade`)

원본이 부르는 `create_robot_map_image` 를 감싼다.

원본 도면(`nabil_map.png`)은 SLAM 지도를 따라 그린 것이 아니라 건물 도면이다.
실측 결과 **벽 일치율이 8%** 였고 로봇 위치가 **2~3 m 어긋난다**.

그래서 Nav2 가 실제로 쓰는 pgm 에 로봇 위치를 정확히 찍은 그림을 만들어
도면 아래에 세로로 붙인다. 결과는 **한 장**이다.

```
상단  PLAN (approx)      건물 도면. 구역 번호로 위치 파악
하단  SLAM MAP (exact)   Nav2 지도. 로봇 위치 정확
```

한 장으로 합치는 이유는 원본 워커의 지도 슬롯이 하나뿐이기 때문이다
(카메라 → 지도 → 메시지). 합치면 원본 전송 흐름을 손대지 않아도 된다.

불 위치는 표시하지 않는다. 발견 시점에는 단안 카메라라 거리를 모른다.
불 마커는 HOLD 알림에서만 나온다.

### 3. 알림 게이트 (`install_alert_gate`)

원본의 `telegram_queue` 앞에 `_GatedQueue` 를 끼운다.

원본 워커 스레드는 시작할 때 진짜 큐 객체를 인자로 붙잡았고
(`args=(telegram_queue,)`), 원본 메인 루프는 매번 전역을 다시 조회한다
(`telegram_queue.put_nowait(...)`). 그래서 전역만 바꾸면

```
메인 루프 -> 게이트 -> (통과 시) 진짜 큐 -> 워커
```

가 되어 중간에 낄 수 있다.

**동작**

| 상황 | 결과 |
|---|---|
| 순찰 중 (`fire_episode=false`) | 통과 — 이게 "발견 즉시 알림" 이다 |
| 화재 처리 중, 첫 번째 | 통과 — 한 건당 최소 1회 보장 |
| 화재 처리 중, 두 번째 이후 | 차단 |
| 원본 종료 신호(`None`) | 항상 통과 |

**"한 건당 최소 1회" 를 지키는 이유** — 원본 알림은 MLP 가 독립적으로 띄우
므로 주행 쪽이 `FIRE_STOP` 에 들어간 직후에 나올 수도 있다. 그때 무조건
막으면 그 화재는 통보가 아예 안 간다. 공장이 타는 동안 아무도 모르는 상황이
제일 나쁘다.

`suppress_original_alert:=true` 면 게이트를 처음부터 닫아 원본 알림을
완전히 막는다. 그 경우 알림은 `send_hold_alert:=true` 로만 나간다.

### 4. 전송 직렬화 (`serialize_telegram`)

`send_telegram_photo` / `send_telegram_message` 를 `TELEGRAM_LOCK`(RLock)
으로 감싼다.

전송 주체가 둘이라 (원본 워커 스레드, 우리 `_send_fire_alert` 스레드)
동시에 `requests.post` 를 날리면 수신함에서 **사진 순서가 뒤섞인다**.
실제 시험에서 그렇게 나왔다. 락으로 한 알림을 한 덩어리로 묶는다.

`send_hold_alert:=false` 인 기본 설정에서는 전송자가 하나뿐이라 락이
경합하지 않는다. `true` 로 켤 때를 위한 보험이다.

## HOLD 알림 (기본 off)

주행 쪽이 불 0.6 m 앞(`HOLD`)에 도달하면 `/argos/fire_alert_request` 로
좌표를 보낸다. `send_hold_alert:=true` 면 별도 스레드에서 다음을 보낸다.

```
1) 현장 사진 (YOLO 박스)
2) 건물 도면
3) 실제 SLAM 지도 (로봇 + 불 마커 + 방향 화살표)
4) 텍스트 (화재 좌표, 로봇 좌표, 전방 거리)
```

**HOLD 좌표가 정확한 이유** — `HOLD` 는 LiDAR 전방 여유가
`fire_stop_dist`(0.6 m) 이하가 됐을 때 들어온다. 즉 "불이 정면 0.6 m 앞"
이 실측으로 확정된다. 단안 카메라의 거리 추정이 필요 없다.

기본이 `false` 인 이유는 알림이 한 건에 두 번 가기 때문이다. 원본 알림이
이미 정확한 SLAM 지도를 싣고 있으므로(위 2번), 대개 그걸로 충분하다.

**`send_hold_alert:=false` 여도 `/main/fire_target` 토픽은 발행된다.**
팔로워봇은 텔레그램이 아니라 그 토픽으로 좌표를 받는다.

## 원본에서 읽어 쓰는 전역

없으면 즉시 죽는 것 (`REQUIRED_GLOBALS`)

```
model   cap   results
```

없어도 되는 것 (`OPTIONAL_GLOBALS`) — 참고/로그용이라 없으면 `null` 로 발행

```
yolo_confs   best_detections   fire_probability   sensor_connected
gas_raw   ir_temperature   danger_duration   last_alert_time
```

함수/객체로 가져다 쓰는 것

```
create_robot_map_image   send_telegram_photo   send_telegram_message
telegram_queue   map_pose_node   SAVE_DIR   annotated_frame   frame
ALERT_COOLDOWN_SECONDS   FIRE_PROB_THRESHOLD
```

인지팀이 이 이름들을 바꾸면 해당 기능이 죽는다. `verify_globals` 가 기동
시점에 확인하고, 없으면 조용히 넘어가지 않고 에러를 남긴다.

## 기동 시 확인할 로그

```
[MAIN] 텔레그램 쿨다운 변경: 60s -> 90.0s (원본 파일은 수정하지 않고 실행 중에만 적용)
[MAIN] 알림 지도 개선: 건물 도면 + 실제 SLAM 지도를 한 장으로 합쳐 보냅니다.
[MAIN] 알림 게이트 설치: 발견 즉시 1회만 보내고, 접근하는 동안의 중복은 막습니다.
[MAIN] 텔레그램: 원본 알림만 보냅니다 (HOLD 지점에서는 /main/fire_target 좌표만 발행)
[MAIN] 텔레그램 전송을 직렬화했습니다 (사진 순서 고정).
[MAIN] HOLD 지점 알림 요청 수신 준비 완료
```

`Fire=... | MLP=...` 줄이 흐르면 인지가 정상 동작 중이다.
TensorRT 워밍업에 6~7 초 걸린다.

## 진단

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic hz /argos/fire_detection
```

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic echo /argos/fire_detection --once
```

```bash
source ~/argos_project/scripts/argos_env.sh && ros2 topic echo /argos/fire_episode
```

장치 점유 확인 (카메라와 아두이노는 이 프로세스가 독점한다)

```bash
sudo fuser -v /dev/video0 /dev/ttyACM0
```

## 알려진 한계

**MLP 성능** (`training_result.txt`, 2026-08-25 재학습, threshold 0.70)

```
Precision 1.000   오경보 없음
Recall    0.667   위험 상황의 약 1/3 은 알림이 나가지 않는다
```

**전송 실패 시 재시도가 없다.** 원본은 큐에 넣는 시점부터 쿨다운을 시작한다.

```python
telegram_queue.put_nowait(alert_job)
last_alert_time = now     # 성공이 아니라 등록 시점
```

전송이 실패해도 다음 시도는 쿨다운(90 초) 뒤다. 5 초 재시도 같은 것을
넣으려면 원본을 고쳐야 한다.

**도면과 SLAM 지도의 정렬은 맞추지 못했다.** 자동 정렬(bbox 기반)을
시도했으나 겹침이 8.0% → 8.3% 로 거의 개선되지 않았다. 도면이 건물
설계도라 SLAM 지도와 대응점이 없다. 그래서 두 장을 같이 보내는 쪽을
택했다. 도면에서 정확한 위치를 읽으려면 대응점 2 개 이상을 사람이
지정해 변환을 구해야 한다.

**불 위치는 발견 시점에 알 수 없다.** 단안 카메라는 방위각만 준다.
`HOLD` 에 도달해야 거리가 확정된다.

## 원복

```bash
cp ~/argos_project/scripts/fire_perception_main.py.bak_20260831_204133 ~/argos_project/scripts/fire_perception_main.py
```

이 파일 자체를 지워도 원본 인지는 그대로 동작한다.

```bash
rm ~/argos_project/scripts/fire_perception_main.py
```
