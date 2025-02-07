from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_prefix

def generate_launch_description():
    # Constants for paths and robot configuration
    package_name = 'robot_description'
    robot_name_in_model = 'ajrobot'
    urdf_file_path = 'urdf/ajrobot.xacro'
    world_package_name = 'simulation_world'
    world_file_path = 'worlds/hospital_area.world'
    rviz_config_file_path = 'rviz/urdf_config.rviz'

    # Set the path to different files and folders
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_pkg_share = FindPackageShare(package=world_package_name).find(world_package_name)
    
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(world_pkg_share, world_file_path)

    # Set GAZEBO_MODEL_PATH
    models_path = os.pathsep + os.path.join(os.path.expanduser('~'), 'models')
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += models_path + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path + pkg_share_path

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to world model file'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])
        }],
        arguments=[default_urdf_model_path]
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 30,
        }]
    )

    # Spawn robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create and return launch description
    return LaunchDescription([
        # Launch arguments
        declare_model_path_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_headless_cmd,
        declare_world_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_rviz_cmd,
        
        # Nodes and processes
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_cmd,
        start_rviz_cmd
    ])