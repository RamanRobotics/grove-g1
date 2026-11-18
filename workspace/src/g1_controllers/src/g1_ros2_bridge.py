#!/usr/bin/env python3
"""
g1_cmd_vel_bridge.py
Subscribe to geometry_msgs/Twist on /cmd_vel and drive the Unitree G1
through the built-in high-level locomotion service (LocoClient.Move).

Usage:
  python3 g1_cmd_vel_bridge.py --ros-args -p network_interface:=eth0

Requirements:
  - ROS 2 (rclpy, geometry_msgs)
  - unitree_sdk2_python installed (pip install -e . in the repo)
  - Robot in Main Operation Control (R1+X on the remote), NOT debug mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def clamp(value, limit):
    return max(-limit, min(limit, value))


class G1CmdVelBridge(Node):
    def __init__(self):
        super().__init__("g1_cmd_vel_bridge")

        # Parameters (start conservative; raise limits once you trust the setup)
        self.declare_parameter("network_interface", "eth0")
        self.declare_parameter("max_vx", 0.5)      # m/s
        self.declare_parameter("max_vy", 0.3)      # m/s
        self.declare_parameter("max_vyaw", 0.8)    # rad/s
        self.declare_parameter("rate_hz", 20.0)    # how often Move() is sent
        self.declare_parameter("cmd_timeout", 0.5) # s without cmd_vel -> stop

        iface = self.get_parameter("network_interface").value
        self.max_vx = self.get_parameter("max_vx").value
        self.max_vy = self.get_parameter("max_vy").value
        self.max_vyaw = self.get_parameter("max_vyaw").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        rate_hz = self.get_parameter("rate_hz").value

        # Unitree DDS + locomotion client
        ChannelFactoryInitialize(0, iface)
        self.loco = LocoClient()
        self.loco.SetTimeout(10.0)
        self.loco.Init()
        self.get_logger().info(f"LocoClient initialized on interface '{iface}'")

        self.vx = self.vy = self.vyaw = 0.0
        self.last_cmd_time = None
        self.stopped = True

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate_hz, self.on_timer)

    def on_cmd_vel(self, msg: Twist):
        self.vx = clamp(msg.linear.x, self.max_vx)
        self.vy = clamp(msg.linear.y, self.max_vy)
        self.vyaw = clamp(msg.angular.z, self.max_vyaw)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        # Watchdog: if commands stop arriving, command zero velocity once
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            if not self.stopped:
                self.send(0.0, 0.0, 0.0)
                self.stopped = True
                self.get_logger().warn("cmd_vel timeout -> zero velocity")
            return

        self.send(self.vx, self.vy, self.vyaw)
        self.stopped = False

    def send(self, vx, vy, vyaw):
        try:
            ret = self.loco.Move(vx, vy, vyaw)
            if ret not in (0, None):
                self.get_logger().warn(f"Move() returned error code {ret}")
        except Exception as e:
            self.get_logger().error(f"Move() failed: {e}")

    def shutdown(self):
        self.send(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    node = G1CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
