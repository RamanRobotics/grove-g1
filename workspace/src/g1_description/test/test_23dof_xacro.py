# Validates the 23-DOF xacro with each hand: the body component, the onboard legs, the hands.
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

LEGS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
]
ARMS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll")
]
UPPER = ["waist_yaw_joint", *ARMS]

O6_ACTUATED = [
    "thumb_cmc_pitch",
    "thumb_cmc_yaw",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
]
O6_MIMICS = ["thumb_ip", "index_dip", "middle_dip", "ring_dip", "pinky_dip"]


def _expand(*args):
    result = subprocess.run(
        ["xacro", os.environ["G1_23DOF_XACRO"], *args], check=True, capture_output=True, text=True
    )
    return ET.fromstring(result.stdout)


@pytest.fixture(scope="module", params=["dummy", "dex3", "linkerbot_o6"])
def robot(request):
    return request.param, _expand(f"hand:={request.param}")


def components(root):
    return {c.get("name"): c for c in root.findall("ros2_control")}


def test_body_component_holds_the_23_joints_in_order(robot):
    _, root = robot
    body = components(root)["G1ArmSdkSystem"]
    assert body.find("hardware/plugin").text == "g1_hardware_interface/G1ArmSdkSystem"
    assert [j.get("name") for j in body.findall("joint")] == LEGS[:6] + LEGS[6:] + UPPER


def test_legs_stay_with_the_onboard_controller(robot):
    """A leg command interface would let a controller fight the onboard balance."""
    _, root = robot
    body = components(root)["G1ArmSdkSystem"]
    for joint in body.findall("joint"):
        commands = [c.get("name") for c in joint.findall("command_interface")]
        if joint.get("name") in LEGS:
            assert commands == [], joint.get("name")
        else:
            assert commands == ["position", "velocity", "effort", "kp", "kd"], joint.get("name")
            params = {p.get("name") for p in joint.findall("param")}
            assert {"position_only_kp", "position_only_kd"} <= params


def test_loco_gpio_and_imu(robot):
    _, root = robot
    body = components(root)["G1ArmSdkSystem"]
    gpio = body.find("gpio")
    assert gpio.get("name") == "loco"
    assert [c.get("name") for c in gpio.findall("command_interface")] == [
        "linear_x",
        "linear_y",
        "angular_z",
    ]
    assert body.find("sensor").get("name") == "imu"


def test_every_body_joint_is_in_the_kinematic_tree(robot):
    _, root = robot
    tree = {j.get("name") for j in root.findall("joint")}
    assert set(LEGS + UPPER) <= tree


def test_hand_components_match_the_hand(robot):
    hand, root = robot
    names = set(components(root)) - {"G1ArmSdkSystem"}
    expected = {
        "dummy": set(),
        "dex3": {"G1Dex3SystemLeft", "G1Dex3SystemRight"},
        "linkerbot_o6": {"LinkerBotO6SystemLeft", "LinkerBotO6SystemRight"},
    }[hand]
    assert names == expected


def test_o6_wire_order_and_mimics():
    root = _expand("hand:=linkerbot_o6")
    for side, prefix in (("Left", "lh"), ("Right", "rh")):
        hand = components(root)[f"LinkerBotO6System{side}"]
        joints = [j.get("name") for j in hand.findall("joint")]
        assert joints == [f"{prefix}_{j}" for j in O6_ACTUATED + O6_MIMICS]
        for joint in hand.findall("joint")[len(O6_ACTUATED) :]:
            assert joint.findall("command_interface") == []
            params = {p.get("name"): p.text for p in joint.findall("param")}
            assert params["mimic_parent"].startswith(prefix + "_")


def test_o6_can_interface_override():
    root = _expand("hand:=linkerbot_o6", "o6_can_interface:=vcan0")
    hand = components(root)["LinkerBotO6SystemLeft"]
    params = {p.get("name"): p.text for p in hand.findall("hardware/param")}
    assert params["can_interface"] == "vcan0"
    assert params["can_id"] == "40"


def test_unknown_hand_fails():
    result = subprocess.run(
        ["xacro", os.environ["G1_23DOF_XACRO"], "hand:=paw"], capture_output=True, text=True
    )
    assert result.returncode != 0
    assert "not a hand" in result.stderr
