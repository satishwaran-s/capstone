import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Common parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    params_file = LaunchConfiguration('params_file')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('capstone_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'spray_paint_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, 
                                        ' use_ros2_control:=', use_ros2_control, 
                                        ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # SLAM toolbox node with explicit parameters
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'mode': 'mapping',
                'use_sim_time': use_sim_time,
                'max_laser_range': 8.0,
                'min_laser_range': 0.4,  # Increased to filter close noise
                'map_update_interval': 0.5,
                'resolution': 0.075,
                'max_covariance': 0.01,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.,
                'stack_size_to_use': 40000000,
                'scan_buffer_size': 10,
                'scan_topic': '/scan',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'do_loop_closing': True,
                'loop_serach_distance': 3.0,
                'smear_deviation': 0.03,
                'ramge_threshold': 8.0,
                'loop_match_minimum_chain_size': 5,
                'use_scan_matching': True,
                'scan_to_map_refinement': True,
                'scan_matching_score_threshold': 0.3

            }
        ]
    )

    # Static Transform Publisher for map -> odom
    node_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Optional odometry broadcaster
    odom_broadcaster_node = Node(
        package='capstone_description',
        executable='motor_control_ros2.py',
        name='motor_control_ros2',
        output='screen'
    )

    # Launch description with all declarations and nodes
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('capstone_description'), 
                                      'config', 'mapper_params_online_async.yaml'),
            description='Full path to the parameter file'
        ),

        # Add the nodes to the launch description
        node_robot_state_publisher,
        slam_node,
        # node_static_transform,  # Uncommented to ensure map->odom transform
        # odom_broadcaster_node,  # Keep commented unless you need it
    ])

