import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    your_pkg = get_package_share_directory('your_package')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', 'my_robot', '-file', os.path.join(your_pkg, 'urdf', 'your_model.xacro')],
        ),
        Node(
            package='your_package', executable='ros2_node_to_monitor_and_apply_force.py', output='screen'),
    ])
