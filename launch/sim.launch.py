import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit  # harmonic specific
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    model = LaunchConfiguration('model', default='copterPIX')

    pkg_name = 'drone'
    pkg_path = FindPackageShare(pkg_name)
    xacro_file = PathJoinSubstitution([pkg_path, 'models', model, 'robot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description}

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', model,
                   '-allow_renaming', 'false'
                   ],
        output='screen'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_path, 'worlds'])
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value='copterPIX',
            description='Model to launch'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[params],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
            ),
        ),
        gz_spawn_entity,
        gazebo_resource_path,
    ])
