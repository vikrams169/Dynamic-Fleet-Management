from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='dynamic_fleet_management',
        executable='robot_commander'
    )

    ld.add_action(my_node)

    return ld
