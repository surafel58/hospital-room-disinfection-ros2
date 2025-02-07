from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    gazebo_pkg_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    simulation_pkg_share = FindPackageShare('simulation_world').find('simulation_world')

    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='hosp.world',
        description='Gazebo world file name'
    )
    
    # Set paths
    world_path = os.path.join(simulation_pkg_share, 'worlds', 'hosp.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ajrobot.xacro')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            urdf_path
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Set Gazebo model path
    package_name = 'robot_description'
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ajrobot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0',
            '-Y', '0.0',
            '-unpause'
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False}]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ]) 