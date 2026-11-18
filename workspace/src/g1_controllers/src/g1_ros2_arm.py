#!/usr/bin/env python3
"""
g1_upper_body_bridge.py
Joint-command interface for the Unitree G1 upper body (waist + arms) via the
arm SDK topic rt/arm_sdk, while the built-in controller keeps balancing the legs.

  Subscribes: /upper_body/joint_command   sensor_msgs/JointState  (name + position [rad])
  Publishes:  /upper_body/joint_states    sensor_msgs/JointState  (measured upper-body pose)

Usage:
  python3 g1_upper_body_bridge.py --ros-args -p network_interface:=eth0 -p dof:=29

Run it alongside g1_cmd_vel_bridge.py (separate process). Robot must be standing
in Main Operation Control (R1+X), NOT debug mode.

Safety behaviour:
  - On start, seeds every target from the MEASURED pose, then ramps the blend
    weight 0 -> 1 slowly. Never flip the weight abruptly: the arms can snap.
  - Targets are rate-limited (max_joint_vel) so a big jump in the command
    topic can't fling the arms.
  - If commands stop, it HOLDS the last target (no firmware watchdog exists).
  - On shutdown (Ctrl+C) it ramps the weight back to 0, returning the arms to
    the built-in controller.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# G1 motor indices (unitree_hg). Index 29 is kNotUsedJoint = blend weight.
JOINTS_29DOF = {
    "waist_yaw": 12, "waist_roll": 13, "waist_pitch": 14,
    "left_shoulder_pitch": 15, "left_shoulder_roll": 16, "left_shoulder_yaw": 17,
    "left_elbow": 18, "left_wrist_roll": 19, "left_wrist_pitch": 20,
    "left_wrist_yaw": 21,
    "right_shoulder_pitch": 22, "right_shoulder_roll": 23, "right_shoulder_yaw": 24,
    "right_elbow": 25, "right_wrist_roll": 26, "right_wrist_pitch": 27,
    "right_wrist_yaw": 28,
}
# 23-DoF: single waist yaw, no wrist pitch/yaw.
JOINTS_23DOF = {k: v for k, v in JOINTS_29DOF.items()
                if k not in ("waist_roll", "waist_pitch",
                             "left_wrist_pitch", "left_wrist_yaw",
                             "right_wrist_pitch", "right_wrist_yaw")}
WEIGHT_INDEX = 29


class G1UpperBodyBridge(Node):
    def __init__(self):
        super().__init__("g1_upper_body_bridge")
        self.declare_parameter("network_interface", "eth0")
        self.declare_parameter("dof", 29)               # 29 or 23
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("kp", 60.0)
        self.declare_parameter("kd", 1.5)
        self.declare_parameter("max_joint_vel", 1.0)    # rad/s target rate limit
        self.declare_parameter("ramp_time", 2.0)        # s for weight 0 <-> 1

        iface = self.get_parameter("network_interface").value
        dof = self.get_parameter("dof").value
        self.joints = JOINTS_29DOF if dof == 29 else JOINTS_23DOF
        self.rate_hz = self.get_parameter("rate_hz").value
        self.dt = 1.0 / self.rate_hz
        self.kp = self.get_parameter("kp").value
        self.kd = self.get_parameter("kd").value
        self.max_step = self.get_parameter("max_joint_vel").value * self.dt
        self.ramp_time = self.get_parameter("ramp_time").value

        ChannelFactoryInitialize(0, iface)
        self.crc = CRC()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()

        self.low_state = None
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self.on_low_state, 10)

        self.goal = {}       # requested targets from ROS
        self.target = {}     # rate-limited targets actually sent
        self.weight = 0.0
        self.active = False

        self.create_subscription(
            JointState, "upper_body/joint_command", self.on_command, 10)
        self.state_pub = self.create_publisher(
            JointState, "upper_body/joint_states", 10)
        self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            f"Upper-body bridge on '{iface}', {dof}-DoF, joints: {list(self.joints)}")

    # ---------- callbacks ----------
    def on_low_state(self, msg: LowState_):
        self.low_state = msg

    def on_command(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joints:
                self.goal[name] = pos
            else:
                self.get_logger().warn(f"Unknown joint '{name}' ignored",
                                       throttle_duration_sec=5.0)

    def measured(self, name):
        return self.low_state.motor_state[self.joints[name]].q

    def on_timer(self):
        if self.low_state is None:
            self.get_logger().info("Waiting for rt/lowstate...",
                                   throttle_duration_sec=2.0)
            return

        # Seed from measured pose on first cycle
        if not self.active:
            for name in self.joints:
                self.target[name] = self.measured(name)
                self.goal.setdefault(name, self.target[name])
            self.active = True
            self.get_logger().info("Seeded from measured pose; ramping weight up")

        # Ramp weight toward 1
        self.weight = min(1.0, self.weight + self.dt / self.ramp_time)

        # Rate-limit targets toward goals
        for name in self.joints:
            err = self.goal[name] - self.target[name]
            self.target[name] += max(-self.max_step, min(self.max_step, err))

        self.send(self.weight)
        self.publish_state()

    # ---------- helpers ----------
    def send(self, weight):
        self.cmd.motor_cmd[WEIGHT_INDEX].q = weight
        for name, idx in self.joints.items():
            mc = self.cmd.motor_cmd[idx]
            mc.q = self.target[name]
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.arm_pub.Write(self.cmd)

    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joints)
        js.position = [self.measured(n) for n in self.joints]
        js.velocity = [self.low_state.motor_state[i].dq for i in self.joints.values()]
        self.state_pub.publish(js)

    def release(self):
        """Ramp weight back to 0 so the built-in controller retakes the arms."""
        if not self.active:
            return
        self.get_logger().info("Releasing arms (ramping weight to 0)")
        steps = int(self.ramp_time * self.rate_hz)
        start = self.weight
        for i in range(steps + 1):
            self.send(start * (1.0 - i / steps))
            time.sleep(self.dt)


def main():
    rclpy.init()
    node = G1UpperBodyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
