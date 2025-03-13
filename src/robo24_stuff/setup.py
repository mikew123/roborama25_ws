from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robo24_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo24_can_xy_node = robo24_stuff.robo24_can_xy_node:main',
            'sensor_serial_node = robo24_stuff.sensor_serial_node:main',
            'openmv_serial_node = robo24_stuff.openmv_serial_node:main',
            "robo24_wheel_controller_node = robo24_stuff.robo24_wheel_controller_node:main",
            "robo24_teleop_node = robo24_stuff.robo24_teleop_node:main",
            "robo24_diynav_node = robo24_stuff.robo24_diynav_node:main",
            "robo24_diyslam_node = robo24_stuff.robo24_diyslam_node:main",
            "robo24_imu_serial_node = robo24_stuff.robo24_imu_serial_node:main",
            "robo24_watch_serial_node = robo24_stuff.robo24_watch_serial_node:main",
        ],
    },
)
