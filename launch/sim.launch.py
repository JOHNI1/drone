
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='copterPIX')

    pkg_name = 'drone'
    pkg_path = get_package_share_directory(pkg_name)

    # Path to the xacro file example: drone/models/copterPIX/robot.urdf.xacro
    xacro_file = PathJoinSubstitution([
        TextSubstitution(text=pkg_path),
        TextSubstitution(text='models'),
        model,
        TextSubstitution(text='robot.urdf.xacro')
    ])
    print("using the model:", end="")
    print(model)

    # Command to process xacro file
    robot_description_command = Command(['xacro ', xacro_file])

    # # Debug prints
    # print('Package path:', pkg_path)
    # print('Model:', model)
    # print('xacro file:', xacro_file)
    # print('robot_description_command:', robot_description_command)

    # # Generate URDF for debugging
    # xacro_file_path = os.path.join(pkg_path, 'models', 'copterPIX', 'robot.urdf.xacro')
    # doc = xacro.process_file(xacro_file_path)
    # robot_desc = doc.toprettyxml(indent='  ')
    # print('Generated URDF:\n', robot_desc)

    params = {'robot_description': robot_description_command, 'use_sim_time': use_sim_time}


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'model',
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
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', model],
            output='screen'
        ),
    ])




