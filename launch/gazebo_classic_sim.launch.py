import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import xacro





def initializer(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
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
            lines[i] = '  <xacro:property name="using_gazebo_classic" value="1"/>' + '\n'
            break
    # Step 3: Write the modified list of lines back to the file
    with open(xacro_file_path, 'w') as xacro_file:
        xacro_file.writelines(lines)
    
    robot_description = (xacro.process_file(xacro_file_path)).toxml()

    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}



    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[params],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'verbose': 'true'}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', model],
            output='screen'
        ),
    ]



def declare_arguments():
    print("hi1")
    return [
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
    ]




def generate_launch_description():
    return LaunchDescription(
        declare_arguments() + [OpaqueFunction(function=initializer)]
    )