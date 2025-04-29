
# MRW 3/12/2025 added RPlidar from example
# MRW renamed this file roborama25_bringup_launch.py and added roborama25 prefix to files
# MRW 5/26/2025 created life cycle launch file
# MRW copied lc_bringup to add nav2 launch

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

### copied from RPLIDAR C1 example
#import os

#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#added for life cycle support
from launch_ros.actions import LifecycleNode
#added to launch other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # copied from RPLIDAR C1 example
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
#    frame_id = LaunchConfiguration('frame_id', default='laser')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # from nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # # IMU 
    # efk_config = os.path.join(
    #     get_package_share_directory('robo24_localization'),
    #     'config',
    #     'efk_config.yaml'
    #     )
 
    # Get the text of the robot description URDF - robot_stat_publisher does not open a file
    with open('urdfs/roborama25.urdf','r') as infp:
    	robot_desc = infp.read()

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
        

        launch_ros.actions.Node(
            package='roborama25_stuff',
            executable='roborama25_teleop_node',
            name='teleop'
        ),

        ##### MY ROBOT

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':robot_desc,
                }],
        ),

        launch_ros.actions.LifecycleNode(
            package='roborama25_stuff',
            executable='roborama25_front_sensors_node_lc',
            name='front_sensors_node_lc',
            namespace="",
        ),

        launch_ros.actions.LifecycleNode(
            package='roborama25_stuff',
            executable='roborama25_wheel_controller_node_lc',
            name='wheel_controller_node_lc',
            namespace="",
        ),

        launch_ros.actions.LifecycleNode(
            package='roborama25_stuff',
            executable='roborama25_controller_node_lc',
            name='controller_node_lc',
            namespace="",
        ),
        
        launch_ros.actions.Node(
            package="roborama25_stuff",
            executable="roborama25_lifecycle_node_manager",
            parameters=[
                {"front_sensors_node_name": "front_sensors_node_lc"},
                {"wheel_controller_node_name": "wheel_controller_node_lc"},
                {"controller_node_name": "controller_node_lc"},
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/bringup_launch.py']),
            launch_arguments={
                'params_file': 'param/roborama25_params.yaml',
                "map": "maps/6can_course_home_map.yaml",
            }.items()
        ),
        
                
        launch_ros.actions.Node(
            package='roborama25_stuff',
            executable='openmv_serial_node',
            name='openmv_serial'
        ),

        launch_ros.actions.Node(
            package='roborama25_stuff',
            executable='roborama25_can_xy_node',
            name='roborama25_can_xy'
        ),

        # launch_ros.actions.Node(
        #     package='roborama25_stuff',
        #     executable='robo24_diynav_node',
        #     name='robo24_diynav'
        # ),

        # launch_ros.actions.Node(
        #     package='roborama25_stuff',
        #     executable='robo24_diyslam_node',
        #     name='robo24_diyslam'
        # ),

        # launch_ros.actions.Node(
        #     package='roborama25_stuff',
        #     executable='robo24_imu_serial_node',
        #     name='robo24_imu_serial'
        # ),

        # launch_ros.actions.Node(
        #     package='roborama25_stuff',
        #     executable='robo24_watch_serial_node',
        #     name='robo24_watch_serial'
        # ),

        # Ros2 system stuff
        # TODO: remove when integrated into roborama25_teleop
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
