from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    print('Launching ROS2 integration test...')

    thruster_control_publisher_node = Node(
        package = 'py_ros2_holoocean',
        executable = 'talker',
        name = 'talker',
        )

    simulator_node = Node(
        package = 'py_ros2_holoocean',
        executable = 'listener',
        name = 'listener',
        )

    print('Returning LaunchDescription...')

    return LaunchDescription([
        thruster_control_publisher_node,
        simulator_node,
    ])
