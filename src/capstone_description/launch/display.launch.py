import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('capstone_description'))
    xacro_file = os.path.join(pkg_path,'urdf','spray_paint_robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # # Launch the controller_manager node
    # controller_manager_params = {
    #     'use_sim_time': use_sim_time,
    #     'robot_description': robot_description_config,
    #     'controller_manager': os.path.join(get_package_share_directory('capstone_description'), 'config', 'controller.yaml')
    # }
    
    # node_controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     name='controller_manager',
    #     output='screen',
    #     parameters=[controller_manager_params]
    # )


    # # Launch the differential drive controller node
    # node_diff_drive = Node(
    #     package='diff_drive_controller',
    #     executable='diff_drive_controller_node',
    #     name='diff_drive',
    #     output='screen',
    #     parameters=[{
    #         'cmd_vel_topic': '/cmd_vel',
    #         'odom_topic': '/odom',
    #         'wheel_separation': 0.5,
    #         'wheel_radius': 0.1
    #     }]
    # )

    # Static Transform Publisher for map -> odom
    node_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config}]
    # )

    slam_node = Node(
        package= 'slam_toolbox',
        executable= 'sync_slam_toolbox_node',
        name= 'slam_toolbox',
        output= 'screen',
        parameters= ['/home/satishrpi/capstone_ws/src/capstone_description/config/mapper_params_online_async.yaml']
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher,
        # slam_node
        # node_static_transform
        # node_controller_manager,
        # node_diff_drive
    ])
