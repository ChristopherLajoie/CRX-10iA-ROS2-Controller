from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  
    ethernetip_adapter_node = Node(
        package='moveit_controller',
        executable='eip_adapter', 
        name='ethernetip_adapter',
        output='screen'
    )

    return LaunchDescription([
        ethernetip_adapter_node,
    ])