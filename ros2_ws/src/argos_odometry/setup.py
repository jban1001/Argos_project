from setuptools import find_packages, setup

package_name = 'argos_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odyssey',
    maintainer_email='odyssey@example.com',
    description='ARGOS wheel encoder odometry',
    license='Apache-2.0',
    tests_require=['pytest'],

   entry_points={
    'console_scripts': [
        'encoder_serial_node = argos_odometry.encoder_serial_node:main',
        'wheel_odometry_node = argos_odometry.wheel_odometry_node:main',
        'calibrate_odometry = argos_odometry.calibrate_odometry:main',
        'argos_base_driver = argos_odometry.argos_base_driver:main',
        'scan_normalizer = argos_odometry.scan_normalizer:main',
    ],
  },
)

