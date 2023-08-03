from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_state_publisher_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('py_ros2_holoocean').find('py_ros2_holoocean') + '/launch/robot_state_publisher.launch.py'),
    )

    rviz2_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments = [
            '-d', FindPackageShare('py_ros2_holoocean').find('py_ros2_holoocean') + '/launch/holoocean_ros.rviz',
        ],
    )
    
    publisher_subscriber_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('py_ros2_holoocean').find('py_ros2_holoocean') + '/launch/publisher_subscriber.launch.py'),
    )

    return LaunchDescription([
        robot_state_publisher_launch_description,
        rviz2_node,
        publisher_subscriber_launch_description,
    ])