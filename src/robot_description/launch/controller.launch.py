from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    
    # Set paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'ajrobot.xacro')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            urdf_file
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Load controllers configuration from YAML
    controller_params_file = PathJoinSubstitution([
        pkg_share,
        'launch',
        'controller.yaml'
    ])

    # Create robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/ajrobot/joint_states'),
        ],
    )

    # Create controller manager node
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='ajrobot',
        output='screen',
        parameters=[controller_params_file],
        arguments=[
            'rwheel_joint_position_controller',
            'lwheel_joint_position_controller',
            'joint_state_controller'
        ],
    )

    # Make sure the controller spawner starts after the joint state publisher
    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[controller_spawner],
        )
    )

    # Create and return launch description
    return LaunchDescription([
        robot_state_publisher,
        event_handler
    ]) 