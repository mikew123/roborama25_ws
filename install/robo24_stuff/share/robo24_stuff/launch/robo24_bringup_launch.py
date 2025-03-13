
# MRW 1/25/2024 Added robot_localization

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

### copied from RPLIDAR C1 exampple
#import os

#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # copied from RPLIDAR C1 example
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
#    frame_id = LaunchConfiguration('frame_id', default='laser')
    frame_id = LaunchConfiguration('frame_id', default='lidar')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # IMU 
    efk_config = os.path.join(
        get_package_share_directory('robo24_localization'),
        'config',
        'efk_config.yaml'
        )
 
 
    return launch.LaunchDescription([

        ##### copied from RPLIDAR C1 example
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen'),
        

        ##### MY ROBOT
        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='robo24_can_xy_node',
        #     name='robo24_can_xy'
        # ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='sensor_serial_node',
            name='sensor_serial'
        ),

        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='openmv_serial_node',
        #     name='openmv_serial'
        # ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_teleop_node',
            name='robo24_teleop'
        ),

        launch_ros.actions.Node(
            package='robo24_stuff',
            executable='robo24_wheel_controller_node',
            name='robo24_wheel_controller'
        ),

        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='robo24_diynav_node',
        #     name='robo24_diynav'
        # ),

        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='robo24_diyslam_node',
        #     name='robo24_diyslam'
        # ),

        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='robo24_imu_serial_node',
        #     name='robo24_imu_serial'
        # ),

        # launch_ros.actions.Node(
        #     package='robo24_stuff',
        #     executable='robo24_watch_serial_node',
        #     name='robo24_watch_serial'
        # ),

        # Ros2 system stuff
        # TODO: remove when integrated into robo24_teleop
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist',
            parameters=[{
                "enable_button": 9,
                "axis_linear.x": 1,
                "axis_angular.yaw": 0,
            }]
        ),

        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_xbox'
        ),
  
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='efk_odom',
        #     parameters=[efk_config]
        # )
        
    ])
