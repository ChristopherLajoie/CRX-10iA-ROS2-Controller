# global_start.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value='192.168.1.20',
        description='IP address for the moveit_controller node'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='5000',
        description='Port for the moveit_controller node'
    )

    moveit_controller_node = Node(
        package='moveit_controller',
        executable='controller',
        name='moveit_controller',
        output='screen',
        parameters=[
            {'ip_address': LaunchConfiguration('ip_address')},
            {'port': LaunchConfiguration('port')}
        ]
    )

    part_manager_node = Node(
        package='moveit_controller',
        executable='part_manager',
        name='paart_manager',
        output='screen'
    )

    return LaunchDescription([
        ip_address_arg,
        port_arg,
        moveit_controller_node,
        part_manager_node
    ])