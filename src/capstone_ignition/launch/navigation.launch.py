# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import xacro


# def generate_launch_description():
#     pkg_name = "capstone_ignition"
#     map_file = "my_mappy.yaml"

#     # Get package share directory
#     pkg_share = get_package_share_directory(pkg_name)
#     desc_dir = get_package_share_directory("capstone_description")

#     # Load URDF with xacro processing
#     robot_xacro_path = os.path.join(desc_dir, "urdf", "spray_paint_robot.urdf.xacro")
#     doc = xacro.parse(open(robot_xacro_path))
#     xacro.process_doc(doc)
#     robot_description = {"robot_description": doc.toxml()}

#     # Controller config path (Make sure this file exists!)
#     controller_config_path = os.path.join(pkg_share, "params", "controller.yaml")
#     controller_config = {"robot_controller": controller_config_path}

#     use_sim_time = LaunchConfiguration("use_sim_time", default="false")
#     use_rviz = LaunchConfiguration("use_rviz", default="true")

#     map_dir = LaunchConfiguration(
#         "map",
#         default=os.path.join(pkg_share, "maps", map_file),
#     )

#     param_dir = LaunchConfiguration(
#         "params_file",
#         default=os.path.join(pkg_share, "params", "nav2_params.yaml"),
#     )

#     return LaunchDescription(
#         [
#             DeclareLaunchArgument(
#                 "map", default_value=map_dir, description="Full path to map file"
#             ),
#             DeclareLaunchArgument(
#                 "params_file",
#                 default_value=param_dir,
#                 description="Full path to Nav2 parameters file",
#             ),
#             DeclareLaunchArgument(
#                 "use_sim_time",
#                 default_value="false",
#                 description="Use simulation time if true",
#             ),
#
#             Node(
#                 package="robot_state_publisher",
#                 executable="robot_state_publisher",
#                 name="robot_state_publisher",
#                 output="screen",
#                 parameters=[
#                     {"use_sim_time": use_sim_time},
#                     robot_description,
#                 ],
#             ),
#
#             Node(
#                 package="controller_manager",
#                 executable="ros2_control_node",
#                 parameters=[
#                     robot_description,
#                     controller_config,  # Now using correct dictionary format
#                 ],
#                 output="screen",
#             ),
#
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["diff_drive_controller"],
#                 output="screen",
#             ),
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_state_broadcaster"],
#                 output="screen",
#             ),
#
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     os.path.join(
#                         get_package_share_directory("nav2_bringup"),
#                         "launch",
#                         "navigation_launch.py",
#                     )
#                 ),
#                 launch_arguments={
#                     "map": map_dir,
#                     "use_sim_time": use_sim_time,
#                     "params_file": param_dir,
#                     "autostart": "true",
#                 }.items(),
#             ),
#
#             Node(
#                 package="rviz2",
#                 executable="rviz2",
#                 name="rviz2",
#                 arguments=[
#                     "-d",
#                     os.path.join(
#                         get_package_share_directory("nav2_bringup"),
#                         "rviz",
#                         "nav2_default_view.rviz",
#                     ),
#                 ],
#                 parameters=[{"use_sim_time": use_sim_time}],
#                 condition=IfCondition(use_rviz),
#                 output="screen",
#             ),
#         ]
#     )


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "capstone_ignition"  # Change to your package name
    map_file = "my_mappy.yaml"  # Your map file name

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(get_package_share_directory(pkg_name), "maps", map_file),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory(pkg_name), "params", "nav2_params.yaml"
        ),
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=map_dir, description="Full path to map file"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to Nav2 parameters file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
