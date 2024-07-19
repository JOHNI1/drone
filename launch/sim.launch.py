
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='copterPIX')


    # pkg_name = 'drone'
    # pkg_path = get_package_share_directory(pkg_name)

    # # Path to the xacro file example: drone/models/copterPIX/robot.urdf.xacro
    # xacro_file = PathJoinSubstitution([
    #     TextSubstitution(text=pkg_path),
    #     TextSubstitution(text='models'),
    #     model,
    #     TextSubstitution(text='robot.urdf.xacro')
    # ])


    # # Command to process xacro file
    # robot_description_command = Command(['xacro ', xacro_file])


    # params = {'robot_description': robot_description_command, 'use_sim_time': use_sim_time}


    pkg_name = 'drone'
    pkg_path = FindPackageShare(pkg_name)
    xacro_file = PathJoinSubstitution([pkg_path, 'models', model, 'robot.urdf.xacro'])
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
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'verbose': 'true'}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', model],
            output='screen'
        ),
        Node(
            package='drone',
            executable='apply_force_node', 
            name='apply_force_node',
            output='screen'
        ),
    ])





# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
# from launch_ros.actions import Node
# import xacro

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     model = LaunchConfiguration('model', default='copterPIX')

#     pkg_name = 'drone'
#     pkg_path = get_package_share_directory(pkg_name)

#     # Path to the xacro file example: drone/models/copterPIX/robot.urdf.xacro
#     xacro_file_path = PathJoinSubstitution([
#         TextSubstitution(text=pkg_path),
#         TextSubstitution(text='models'),
#         model,
#         TextSubstitution(text='robot.urdf.xacro')
#     ])
#     print("xacro_file_path", end="")
#     print(xacro_file_path)
#     print("model", end="")
#     print(model)

#     xacro_file_path = os.path.join(pkg_path, 'models', model, 'robot.urdf.xacro')

#     xacro_file = xacro.process_file(xacro_file_path)
#     robot_description_xml = xacro_file.toxml()


#     params = {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}


#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='true',
#             description='Use sim time if true'
#         ),
#         DeclareLaunchArgument(
#             'model',
#             default_value='copterPIX',
#             description='Model to launch'
#         ),
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[params],
#             output='screen'
#         ),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
#         ),
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=['-topic', 'robot_description', '-entity', model],
#             output='screen'
#         ),
#     ])




