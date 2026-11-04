"""The MoveIt config for each robot and hand, shared by this package's launch files and by
g1_manipulation's. Not a launch file: importing it starts nothing.

    sys.path.insert(0, os.path.join(get_package_share_directory("g1_moveit_config"), "launch"))
    from g1_moveit_robot import moveit_config
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONFIG_SHARE = get_package_share_directory("g1_moveit_config")

ROBOTS = ("g1_29dof", "g1_23dof")
HANDS = {"g1_29dof": ("dex3",), "g1_23dof": ("dummy", "dex3", "linkerbot_o6")}


def config_path(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def check(robot, hand):
    """The robot and hand, the hand defaulted, or a RuntimeError naming what fits."""
    if robot not in ROBOTS:
        raise RuntimeError(f"robot:={robot!r} is not a robot. Use one of {', '.join(ROBOTS)}.")
    hand = hand or HANDS[robot][0]
    if hand not in HANDS[robot]:
        raise RuntimeError(
            f"hand:={hand!r} does not fit robot:={robot}. It takes {', '.join(HANDS[robot])}."
        )
    return robot, hand


def moveit_config(robot, hand, geometry_only=False, execution=True, sensors=True):
    """Every path explicit: the builder otherwise guesses file names and silently skips a missing
    one.

    geometry_only takes the 29-DOF description without its ros2_control block, for consumers that
    must not look like they claim the motors (RViz). The 23-DOF xacro is the same either way: the
    block is inert outside controller_manager.
    """
    robot, hand = check(robot, hand)
    builder = MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
    if robot == "g1_29dof":
        xacro = "g1_common.xacro" if geometry_only else "g1_lowcmd.urdf.xacro"
        builder = (
            builder.robot_description(file_path=os.path.join(DESCRIPTION_SHARE, "urdf", xacro))
            .robot_description_semantic(file_path=config_path("g1.srdf"))
            .robot_description_kinematics(file_path=config_path("kinematics.yaml"))
            .joint_limits(file_path=config_path("joint_limits.yaml"))
        )
        controllers = config_path("moveit_controllers.yaml")
    else:
        builder = (
            builder.robot_description(
                file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_23dof.urdf.xacro"),
                mappings={"hand": hand},
            )
            .robot_description_semantic(file_path=config_path(f"g1_23dof_{hand}.srdf"))
            .robot_description_kinematics(file_path=config_path("kinematics_23dof.yaml"))
            .joint_limits(file_path=config_path(f"joint_limits_23dof_{hand}.yaml"))
        )
        controllers = config_path(f"moveit_controllers_23dof_{hand}.yaml")
    if execution:
        builder = builder.trajectory_execution(file_path=controllers)
    # Named, or the builder assembles every pipeline it knows and pilz fails the launch wanting a
    # pilz_cartesian_limits.yaml this config does not ship.
    builder = builder.planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
    if sensors:
        builder = builder.sensors_3d(file_path=config_path("sensors_3d.yaml"))
    return builder.to_moveit_configs()
