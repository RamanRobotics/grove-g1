"""RViz with the MotionPlanning panel, for dragging the arms around by hand.

Its own launcher rather than g1_bringup's rviz.launch.py because the panel needs MoveIt's
kinematics and planning parameters on its own node. Run alongside a stack that is already up;
executing also needs `ros2 launch g1_bringup activate_arm.launch.py`.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_SHARE = get_package_share_directory("g1_moveit_config")
sys.path.insert(0, os.path.join(CONFIG_SHARE, "launch"))
from g1_moveit_robot import check, moveit_config  # noqa: E402


def _config(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def _rviz(context, *args, **kwargs):
    """The shared description, not the stack's own: RViz reads links and joints and never the
    ros2_control block, so this stays correct however the hardware side is configured."""
    robot, hand = check(
        LaunchConfiguration("robot").perform(context), LaunchConfiguration("hand").perform(context)
    )
    config = moveit_config(robot, hand, geometry_only=True, execution=False, sensors=False)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                config.joint_limits,
                config.planning_pipelines,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=_config("g1_moveit.rviz"),
                description="RViz config to open. The default starts on the both_arms group.",
            ),
            DeclareLaunchArgument("robot", default_value="g1_29dof", description="As move_group."),
            DeclareLaunchArgument("hand", default_value="", description="As move_group."),
            OpaqueFunction(function=_rviz),
        ]
    )
