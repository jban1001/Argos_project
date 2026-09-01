"""런치 파일이 조립되는지 확인한다. 노드를 띄우지는 않는다.

경로 오타나 없는 패키지는 실행해봐야 드러나는데, 그때는 이미 카메라와 시리얼
포트를 물고 있는 상태라 되돌리기가 번거롭다. 여기서 미리 잡는다.
"""

from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions

SHARE = Path(get_package_share_directory("follower_bringup"))
LAUNCH_FILES = sorted(p.name for p in (SHARE / "launch").glob("*.launch.py"))


def load(name: str) -> LaunchDescription:
    source = PythonLaunchDescriptionSource(str(SHARE / "launch" / name))
    return source.get_launch_description(LaunchContext())


def test_every_launch_file_is_discovered():
    """런치를 추가하고 setup.py 의 glob 에서 빠뜨리면 설치되지 않는다."""
    assert LAUNCH_FILES, "설치된 런치 파일이 없다"
    for expected in ("camera.launch.py", "serial_bridge.launch.py",
                     "aruco_pose.launch.py", "cooperative.launch.py",
                     "follow.launch.py", "follower.launch.py"):
        assert expected in LAUNCH_FILES, f"{expected} 가 설치되지 않았다"


@pytest.mark.parametrize("name", LAUNCH_FILES)
def test_launch_description_builds(name):
    description = load(name)
    assert isinstance(description, LaunchDescription)
    assert description.entities, f"{name} 이 비어 있다"


def test_integration_launch_declares_every_component():
    """빠진 인자가 있으면 그 노드를 켤 방법이 없다."""
    description = load("follower.launch.py")
    declared = {entity.name for entity in description.entities
                if hasattr(entity, "name") and entity.name}
    for expected in ("camera", "serial_bridge", "vio", "aruco",
                     "cooperative", "follow", "publish_commands"):
        assert expected in declared, f"'{expected}' 인자가 없다"


def test_every_argument_has_a_description():
    """설명 없는 인자는 --show-args 에서 'no description given' 으로 나와
    아무 도움이 안 된다."""
    description = load("follower.launch.py")
    missing = [entity.name for entity in description.entities
               if hasattr(entity, "_description") and hasattr(entity, "name")
               and entity.name and not entity._description]
    assert not missing, f"설명 없는 인자: {missing}"


def test_motor_commands_are_off_by_default():
    """실수로 바퀴가 도는 일이 없어야 한다."""
    description = load("follower.launch.py")
    context = LaunchContext()
    for entity in description.entities:
        if getattr(entity, "name", None) == "publish_commands":
            value = perform_substitutions(context, entity.default_value)
            assert value == "false", f"기본값이 {value} 다"
            return
    pytest.fail("publish_commands 인자를 찾지 못했다")


def test_estimator_config_exists_for_vio():
    """vio:=true 로 켰을 때 가리키는 설정이 실제로 있어야 한다."""
    config = Path.home() / "follower_ws" / "config" / "openvins" / "estimator_config.yaml"
    assert config.exists(), f"{config} 가 없다 -- scripts/22 로 생성할 것"
