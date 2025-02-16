from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare all launch arguments
    declare_open_rviz_cmd = DeclareLaunchArgument(
        name='open_rviz',
        default_value='true',
        description='Flag to enable RViz2'
    )

    declare_move_forward_only_cmd = DeclareLaunchArgument(
        name='move_forward_only',
        default_value='false',
        description='Flag to only allow forward movement'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        name='map_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('navigation'), 'maps', 'hosp2.yaml']
        ),
        description='Full path to map yaml file'
    )

    # Create nav2 map server node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )

    # Include AMCL launch file
    amcl_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('navigation'),
                'launch',
                'amcl.launch.py'
            ])
        )
    )

    # Include move_base (Nav2) launch file
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('navigation'),
                'launch',
                'move_base.launch.py'
            ])
        ),
        launch_arguments={
            'move_forward_only': LaunchConfiguration('move_forward_only')
        }.items()
    )

    # RViz2
    rviz_cmd = GroupAction(
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('navigation'),
                    'rviz',
                    'ajrobot.rviz'
                ])],
                output='screen'
            )
        ]
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_open_rviz_cmd)
    ld.add_action(declare_move_forward_only_cmd)
    ld.add_action(declare_map_file_cmd)

    # Add nodes and launch files
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld 