"""move_group on its own: planning, no simulator.

Nothing here is sim-specific, so it carries to hardware unchanged. Planning needs only joint
states, and nothing here activates a controller. robot and hand pick the model, as they do for
g1_bringup's control.launch.py, so move_group plans against the robot that is running.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node

CONFIG_SHARE = get_package_share_directory("g1_moveit_config")
sys.path.insert(0, os.path.join(CONFIG_SHARE, "launch"))
from g1_moveit_robot import check, moveit_config  # noqa: E402

# Both arms: matches arm_trajectory_controller's 14 joints, and servo.yaml's move_group_name.
SERVO_GROUP = "both_arms"


def _servo(robot, hand):
    """Off unless asked for. Servo streams onto the same controller topic move_group executes
    through, so running it alongside a planned motion would put two writers on one controller.

    Without sensors_3d: servo would start a second octomap updater on the same sensor, one that
    can disagree with move_group's. It reads move_group's scene over /monitored_planning_scene."""
    return Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config(robot, hand, execution=False, sensors=False).to_dict(),
            # Servo reads its own settings from under this key, not from the node root.
            {
                "moveit_servo": ParameterBuilder("g1_moveit_config")
                .yaml("config/servo.yaml")
                .to_dict()
            },
            # The acceleration-limiting smoother is a separate plugin and takes these at the
            # node root; without them it fails to configure and servo starts with no smoothing.
            {"update_period": 0.01, "planning_group_name": SERVO_GROUP},
        ],
    )


def _move_group(robot, hand):
    # No `name=`: it would remap every node in the process, and move_group's own helper nodes
    # would all become move_group. The executable names itself.
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config(robot, hand).to_dict(),
            # Puts the SRDF on a topic, so RViz's MotionPlanning display picks it up instead of
            # every consumer being handed the same parameter.
            {"publish_robot_description_semantic": True},
            # Off by default, and servo's secondary scene monitor gets its world and robot state
            # from this topic alone; without it servo silently never moves.
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )


def _setup(context, *args, **kwargs):
    robot, hand = check(
        LaunchConfiguration("robot").perform(context), LaunchConfiguration("hand").perform(context)
    )
    actions = [_move_group(robot, hand)]
    if LaunchConfiguration("servo").perform(context).lower() == "true":
        if robot != "g1_29dof":
            raise RuntimeError(
                "servo:=true is configured for the 29-DOF G1's both_arms (config/servo.yaml); the "
                "23-DOF G1 has no servo config."
            )
        actions.append(_servo(robot, hand))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo",
                default_value="false",
                description="Also start MoveIt Servo, the streaming command path for the arms. "
                "Off by default: it and a planned motion would both drive the same controller. "
                "g1_29dof only.",
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="g1_29dof",
                description="g1_29dof or g1_23dof: which model and SRDF to plan against.",
            ),
            DeclareLaunchArgument(
                "hand",
                default_value="",
                description="g1_23dof only: dummy, dex3 or linkerbot_o6. Empty is the robot's "
                "default: dex3 on g1_29dof, dummy on g1_23dof.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
