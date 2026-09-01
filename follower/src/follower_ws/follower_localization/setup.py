from glob import glob
from setuptools import setup

package_name = "follower_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doorian0615",
    maintainer_email="hs12253865@gmail.com",
    description="Cooperative localization for the follower robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "aruco_pose_node = follower_localization.aruco_pose_node:main",
            "cooperative_node = follower_localization.cooperative_node:main",
            "follow_controller_node = follower_localization.follow_controller_node:main",
        ],
    },
)
