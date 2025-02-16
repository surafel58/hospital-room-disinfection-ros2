from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from geometry_msgs.msg import PoseWithCovarianceStamped

def generate_launch_description():
    # Create launch configuration variables
    namespace = LaunchConfiguration('namespace', default='')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Scan topic name'
    )

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': "base_footprint",
            'transform_tolerance': 0.5,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',  # Fixed class name
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_a': 0.0,
            'min_particles': 500,
            'max_particles': 2000,
            'update_min_d': 0.2,
            'update_min_a': 0.2,
            'resample_interval': 1,
            'transform_tolerance': 1.0,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': 0.15,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': scan_topic,
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(amcl_node)

    return ld 