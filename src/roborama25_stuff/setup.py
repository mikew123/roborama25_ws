from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roborama25_stuff'

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
            #'robo24_can_xy_node = roborama25_stuff.robo24_can_xy_node:main',
            'roborama25_sensor_serial_node = roborama25_stuff.roborama25_sensor_serial_node:main',
            #'openmv_serial_node = roborama25_stuff.openmv_serial_node:main',
            "roborama25_wheel_controller_node = roborama25_stuff.roborama25_wheel_controller_node:main",
            "roborama25_teleop_node = roborama25_stuff.roborama25_teleop_node:main",
            #"robo24_diynav_node = roborama25_stuff.robo24_diynav_node:main",
            #"robo24_diyslam_node = roborama25_stuff.robo24_diyslam_node:main",
            #"robo24_imu_serial_node = roborama25_stuff.robo24_imu_serial_node:main",
            #"robo24_watch_serial_node = roborama25_stuff.robo24_watch_serial_node:main",
        ],
    },
)
