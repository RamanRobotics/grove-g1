"""robot_state_publisher, ros2_control_node and the controller spawners.

No simulator, so this carries to hardware unchanged. Two robots:

- g1_29dof: G1LowCmdSystem owns all 29 body motors with no onboard balance underneath, and the
  policy spawned here is the balance controller. Dex3 hands.
- g1_23dof: G1ArmSdkSystem keeps Unitree's onboard locomotion, drives the upper body over
  rt/arm_sdk, and loco_controller hands it /cmd_vel. The hand is dummy (rubber, none), dex3 or
  linkerbot_o6.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONTROLLERS_CONFIG = os.path.join(get_package_share_directory("g1_controllers"), "config")

ROBOTS = ("g1_29dof", "g1_23dof")
# The 29-DOF description carries its Dex3 hands; the 23-DOF one takes any of these.
HANDS = {"g1_29dof": ("dex3",), "g1_23dof": ("dummy", "dex3", "linkerbot_o6")}

XACRO_PATHS = {
    "g1_29dof": os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro"),
    "g1_23dof": os.path.join(DESCRIPTION_SHARE, "urdf", "g1_23dof.urdf.xacro"),
}

# Forwards SIGTERM/INT to the `ros2 run` subprocess, which would otherwise be orphaned, and
# re-waits so launch sees a clean exit.
_SIGNAL_FORWARDING_WRAPPER = (
    "set -m; {command} & child=$!; "
    "trap 'kill -TERM -$child 2>/dev/null; wait $child' TERM INT; "
    "wait $child"
)


# Every body joint is claimed from the start: one the component sees unclaimed is left unpowered.
# The policy and the safety controller it chains into activate as one group, because chained
# reference interfaces only become claimable inside the switch that activates them.
def _controllers(pin_pelvis):
    """The spawn set, differing only in what drives the legs; the freeze and safety controllers
    claim the same 14 joints. The --inactive ones stay loaded for a later switch, such as
    activate_arm's or the safety controller's emergency."""
    legs = (
        [
            (["locomotion_freeze_controller"], []),
            (["locomotion_safety_controller", "agile_controller"], ["--inactive"]),
        ]
        if pin_pelvis
        else [
            (["locomotion_safety_controller", "agile_controller"], ["--activate-as-group"]),
            (["locomotion_freeze_controller"], ["--inactive"]),
        ]
    )
    return [
        (["joint_state_broadcaster"], []),
        (["imu_sensor_broadcaster"], []),
        (["waist_freeze_controller"], []),
        (["arm_freeze_controller"], []),
        *legs,
        (["arm_trajectory_controller"], ["--inactive"]),
        (["left_hand_controller"], ["--inactive"]),
        (["right_hand_controller"], ["--inactive"]),
    ]


def _controllers_23dof(hand):
    """No leg controller: the onboard one walks. loco_controller claims no joint, so it is free to
    start alongside the freezes. The hands wait for activate_arm, as on the 29-DOF G1."""
    hands = (
        []
        if hand == "dummy"
        else [
            (["left_hand_controller"], ["--inactive"]),
            (["right_hand_controller"], ["--inactive"]),
        ]
    )
    return [
        (["joint_state_broadcaster"], []),
        (["imu_sensor_broadcaster"], []),
        (["waist_freeze_controller"], []),
        (["arm_freeze_controller"], []),
        (["loco_controller"], []),
        (["arm_trajectory_controller"], ["--inactive"]),
        *hands,
    ]


def _controller_files(robot, hand):
    """The params files controller_manager loads, later ones winning."""
    if robot == "g1_29dof":
        return [os.path.join(CONTROLLERS_CONFIG, "lowcmd_controllers.yaml")]
    files = [os.path.join(CONTROLLERS_CONFIG, "armsdk_controllers.yaml")]
    if hand != "dummy":
        files.append(os.path.join(CONTROLLERS_CONFIG, f"hand_{hand}_controllers.yaml"))
    return files


def _spawners(controllers):
    return [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", *names, *extra],
            name=f"{names[0]}_spawner",
            output="screen",
        )
        for names, extra in controllers
    ]


def _xacro_args(robot, hand, o6_can_interface):
    if robot == "g1_29dof":
        return []
    args = [f"hand:={hand}"]
    if o6_can_interface:
        args.append(f"o6_can_interface:={o6_can_interface}")
    return args


def _robot_state_publisher(robot, xacro_args):
    description = Command(
        [FindExecutable(name="xacro"), " ", XACRO_PATHS[robot]]
        + [item for arg in xacro_args for item in (" ", arg)]
    )
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
    )


def _control_node(files, overrides):
    """`ros2 run` rather than launch_ros Node: under Node, arm_trajectory_controller's
    parameters reliably declare empty. A later params file wins, so overrides go last."""
    files = files + ([overrides] if overrides else [])
    command = (
        "ros2 run controller_manager ros2_control_node --ros-args "
        "-r '~/robot_description:=/robot_description' "
        + " ".join(f"--params-file {path}" for path in files)
    )
    return ExecuteProcess(
        cmd=["bash", "-c", _SIGNAL_FORWARDING_WRAPPER.format(command=command)],
        name="ros2_control_node",
        output="screen",
    )


def _check_robot(robot, hand, pin_pelvis):
    if robot not in ROBOTS:
        raise RuntimeError(f"robot:={robot!r} is not a robot. Use one of {', '.join(ROBOTS)}.")
    if hand not in HANDS[robot]:
        raise RuntimeError(
            f"hand:={hand!r} does not fit robot:={robot}. It takes {', '.join(HANDS[robot])}."
        )
    if pin_pelvis and robot != "g1_29dof":
        raise RuntimeError(
            "pin_pelvis:=true freezes the 29-DOF G1's policy-driven legs. The 23-DOF G1 walks on "
            "its onboard controller and has no legs here to freeze; drop it."
        )


def _launch_setup(context, *args, **kwargs):
    # A welded pelvis gives the policy observations its own actions cannot move, so it diverges.
    pin_pelvis = LaunchConfiguration("pin_pelvis").perform(context).lower() in ("true", "1")
    robot = LaunchConfiguration("robot").perform(context)
    hand = LaunchConfiguration("hand").perform(context) or HANDS.get(robot, ("",))[0]
    _check_robot(robot, hand, pin_pelvis)

    controllers = _controllers(pin_pelvis) if robot == "g1_29dof" else _controllers_23dof(hand)
    control_node = _control_node(
        _controller_files(robot, hand),
        LaunchConfiguration("controllers_overrides").perform(context),
    )
    xacro_args = _xacro_args(robot, hand, LaunchConfiguration("o6_can_interface").perform(context))
    return [
        _robot_state_publisher(robot, xacro_args),
        control_node,
        *_spawners(controllers),
        # Tear down the whole launch if controller_manager dies.
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pin_pelvis",
                default_value="false",
                description="SIM-ONLY, set by sim.launch.py: freeze the legs instead of running "
                "the balance policy.",
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="g1_29dof",
                description="g1_29dof: all 29 motors on rt/lowcmd under the AGILE policy, Dex3 "
                "hands. g1_23dof: Unitree's onboard locomotion, the upper body on rt/arm_sdk.",
            ),
            DeclareLaunchArgument(
                "hand",
                default_value="",
                description="g1_23dof only: dummy (the rubber hands, no hand component), dex3 or "
                "linkerbot_o6. Empty is the robot's default: dex3 on g1_29dof, dummy on g1_23dof.",
            ),
            DeclareLaunchArgument(
                "o6_can_interface",
                default_value="",
                description="hand:=linkerbot_o6 only: the CAN interface the hands are on. Empty "
                "is g1_description's linkerbot_o6_params.yaml (can0); the simulator uses vcan0.",
            ),
            DeclareLaunchArgument(
                "controllers_overrides",
                default_value="",
                description="A params file layered over g1_controllers' controller config, for "
                "tests such as test_emergency_freeze. Empty adds none.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
