from setuptools import find_packages, setup

package_name = "follower_fire_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="doorian0615",
    maintainer_email="hs12253865@gmail.com",
    description="Follower fire-response mission supervisor",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fire_supervisor_node = follower_fire_control.fire_supervisor_node:main",
        ],
    },
)
