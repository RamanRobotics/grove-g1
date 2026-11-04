"""g1_arm_motion_server: SetArmPosture, MoveHand, SetGrip and GetPostures over MoveIt.

For any robot and hand: robot and hand pick the same MoveIt config move_group plans with. Starts
neither the simulator nor move_group; g1_bringup launches both alongside this.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("g1_manipulation")
sys.path.insert(0, os.path.join(get_package_share_directory("g1_moveit_config"), "launch"))
from g1_moveit_robot import check, moveit_config  # noqa: E402


def _server(context, *args, **kwargs):
    robot, hand = check(
        LaunchConfiguration("robot").perform(context), LaunchConfiguration("hand").perform(context)
    )
    return [
        Node(
            package="g1_manipulation",
            executable="g1_arm_motion_server",
            output="screen",
            parameters=[
                # MoveGroupInterface reads the model from its own node's parameters.
                moveit_config(robot, hand, sensors=False).to_dict(),
                os.path.join(SHARE, "config", "g1_arm_motion_server.yaml"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="g1_29dof", description="As move_group."),
            DeclareLaunchArgument("hand", default_value="", description="As move_group."),
            OpaqueFunction(function=_server),
        ]
    )
