import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    declare_use_ros2_control = DeclareLaunchArgument(
            name='use_ros2_control', default_value=use_ros2_control, description='Use ros2_control if true'
        )

    desc_dir = get_package_share_directory('capstone_description')
    sim_dir = get_package_share_directory('capstone_ignition')

    world = os.path.join(sim_dir, "worlds/turtlebot_world.sdf")
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': [world, ' -r -v1']}.items(),
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(ign_gazebo_node)

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    ld.add_action(clock_bridge)

    robot_xacro_path = os.path.join(desc_dir, "urdf/spray_paint_robot.urdf.xacro")
    doc = xacro.parse(open(robot_xacro_path))
    xacro.process_doc(
        doc,
        mappings={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            }
        )
    robot_description = {"robot_description": doc.toxml()}

    # Create state publisher node for that instance
    robot_state_publisher = Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # Create spawn call
    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=['-topic', "robot_description",
                '-name', 'spray_bot',
                '-x', '-1.5',
                '-y', '-0.5',
                '-z', '0.01'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        output='screen'
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)

    return ld
