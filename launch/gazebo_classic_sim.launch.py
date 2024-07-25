
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='copterPIX')
    pkg_name = 'drone'
    pkg_path = FindPackageShare(pkg_name)
    xacro_file = PathJoinSubstitution([pkg_path, 'models', model, 'robot.urdf.xacro'])


    search_string = '<xacro:property name="using_gazebo_classic"'
    with open(xacro_file, 'r') as file:
        lines = file.readlines()
    for i, line in enumerate(lines):
        if search_string in line:
            lines[i] = '      <xacro:property name="using_gazebo_classic" value="1"/>' + '\n'
            break
    with open(xacro_file, 'w') as file:
        file.writelines(lines)
    
        
    robot_description_command = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_command, 'use_sim_time': use_sim_time}

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
            PythonLaunchDescriptionSource([os.path.join(FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py')])
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', model],
            output='screen'
        ),
    ])