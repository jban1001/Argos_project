from setuptools import find_packages, setup

package_name = 'argos_main_share'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odyssey',
    maintainer_email='odyssey@example.com',
    description='Main Robot -> Follower 공유 계층',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_share_node = argos_main_share.main_share_node:main',
        ],
    },
)
