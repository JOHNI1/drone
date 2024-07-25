import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit  # harmonic specific
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import xacro



def initializer(context):

    model = LaunchConfiguration('model').perform(context)
    print("model: " + model.perform(context))
    pkg_name = 'drone'
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file_path = os.path.join(pkg_path, 'models', model.perform(context), 'robot.urdf.xacro')


    # Step 1: Read the contents of the xacro_file into a list of lines
    search_string = '<xacro:property name="using_gazebo_classic"'
    with open(xacro_file_path, 'r') as xacro_file:
        lines = xacro_file.readlines()
    # Step 2: Search for the target line and replace it
    for i, line in enumerate(lines):
        if search_string in line:
            lines[i] = '  <xacro:property name="using_gazebo_classic" value="0"/>' + '\n'
            break
    # Step 3: Write the modified list of lines back to the file
    with open(xacro_file_path, 'w') as xacro_file:
        xacro_file.writelines(lines)
    
    robot_description = (xacro.process_file(xacro_file_path)).toxml()
    
    params = {'robot_description': robot_description}

    return [
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
        Node(
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
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                PathJoinSubstitution([pkg_path, 'worlds'])
            ]
        ),
    ]

def declare_arguments():
    return [
        DeclareLaunchArgument(
            name='model',
            default_value='copterPIX',
            description='Model to launch'
        )
    ]

def generate_launch_description():
    return LaunchDescription(
        declare_arguments() + [OpaqueFunction(function=initializer)]
    )


