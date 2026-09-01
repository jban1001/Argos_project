# Follower robot deployment snapshot

이 디렉터리는 2026-09-02 팔로우봇에서 실제 사용하던 소스와 설정 중 이번
통합에 필요한 부분을 Git으로 보존한 배포 스냅샷이다.

- `src/follower_ws`: 카메라, ArUco, 추종, 시리얼 브리지 ROS 2 패키지
- `src/follower_fire_control`: 화재 미션과 단일 액추에이터 감독기
- `config`, `launch`, `tools`: `lidar_overlay_ws`의 위치추정/Nav2/화재 파일
- `runtime`: 현장 전체 기동 스크립트와 Fast DDS 설정

`build`, `install`, `log`, bag/npz와 Python/pytest 캐시는 생성물이므로 포함하지
않았다. 사용자 모드 사용법과 안전 절차는 `docs/MISSION_MODES.md`를 참고한다.

장비에 적용할 때는 이 소스를 해당 워크스페이스에 복사하고 `colcon build`한
뒤, 기존 전체 기동 절차를 사용한다. 저장소의 파일은 즉시 장비에 배포되거나
모터/펌프를 켜지 않는다.
