# ethernet_start.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ethernet/IP adapter node
    ethernetip_adapter_node = Node(
        package='moveit_controller',
        executable='ethernetip_utils',  # Name of your C++ executable
        name='ethernetip_adapter',
        output='screen'
    )

    return LaunchDescription([
        ethernetip_adapter_node,
    ])