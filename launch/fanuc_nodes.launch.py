from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.100',
        description='IP address of the robot'
    )
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='crx10ia_l',
        description='Type of the robot'
    )

    # Include both fanuc launch files
    fanuc_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fanuc_ros2_driver'),
                'launch',
                'fanuc_interface.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_type': LaunchConfiguration('robot_type'),
            'output': 'screen'
        }.items()
    )

    test_fanuc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fanuc_ros2_driver'),
                'launch',
                'test_fanuc.launch.py'
            )
        ),
        #PythonLaunchDescriptionSource(
            #os.path.join(
                #get_package_share_directory('custom_moveit_config'),
                #'launch',
                #'custom_test.launch.py'
            #)
        #),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_type': LaunchConfiguration('robot_type'),
            'output': 'log'
        }.items()
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_type_arg,
        fanuc_interface_launch,
        test_fanuc_launch
    ])