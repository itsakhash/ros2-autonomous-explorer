"""Start the whole autonomous-exploration stack with one command.

Starts: Gazebo + TurtleBot3 burger, SLAM Toolbox, Nav2, the Twist->TwistStamped
relay, RViz, and (after a short delay) the frontier explorer.

    ros2 launch autonomous_explorer explore.launch.py
    ros2 launch autonomous_explorer explore.launch.py explore:=false   # no auto-exploring
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("autonomous_explorer")
    nav2_dir = get_package_share_directory("nav2_bringup")
    tb3_gz_dir = get_package_share_directory("turtlebot3_gazebo")
    slam_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    explore = LaunchConfiguration("explore")
    explore_delay = LaunchConfiguration("explore_delay")
    rviz = LaunchConfiguration("rviz")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    slam_params = os.path.join(pkg, "config", "slam_params.yaml")
    rviz_config = os.path.join(nav2_dir, "rviz", "nav2_default_view.rviz")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "explore",
            default_value="true",
            description="Start the frontier explorer automatically",
        ),
        DeclareLaunchArgument(
            "explore_delay",
            default_value="25.0",
            description="Seconds to wait before the explorer starts, so "
            "SLAM and Nav2 are ready",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        # (-2.0, -0.5) is the standard TurtleBot3 start, outside the pillar ring.
        DeclareLaunchArgument(
            "x_pose", default_value="-2.0", description="Robot spawn x (m)"
        ),
        DeclareLaunchArgument(
            "y_pose", default_value="-0.5", description="Robot spawn y (m)"
        ),
    ]

    # Gazebo Harmonic + TurtleBot3 burger in the hexagon world.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gz_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    # SLAM Toolbox builds the map and publishes map -> odom. Our params file lowers
    # the minimum travel distance/heading so short exploration goals still add scans.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params,
        }.items(),
    )

    # Nav2 navigation stack (no AMCL / map server: SLAM provides the map).
    # Our params file sends velocity commands to cmd_vel_nav2.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    # Nav2 publishes Twist; the TurtleBot3 bridge wants TwistStamped on /cmd_vel.
    relay = Node(
        package="autonomous_explorer",
        executable="twist_relay",
        name="twist_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="log",
    )

    explorer = TimerAction(
        period=explore_delay,
        actions=[
            Node(
                package="autonomous_explorer",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(explore),
            )
        ],
    )

    return LaunchDescription(args + [sim, slam, nav2, relay, rviz_node, explorer])
