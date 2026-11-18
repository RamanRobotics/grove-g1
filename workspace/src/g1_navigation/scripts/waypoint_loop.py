#!/usr/bin/env python3
"""Drives the robot round a waypoint file, in a loop, one navigate_to_pose goal per waypoint.

Reads the file the Nav2 RViz panel saves and loads (waypoints: waypointN: pose, orientation).
waypoint_follower is not launched in this stack, so this sends each pose to bt_navigator itself:

    ros2 run g1_navigation waypoint_loop.py                     # ces_waypoints.yaml, forever
    ros2 run g1_navigation waypoint_loop.py --loops 3 --pause 2
    ros2 run g1_navigation waypoint_loop.py /path/to/other_waypoints.yaml

Needs localization and nav:=true up. Ctrl-C cancels the goal in flight and stops.
"""

import argparse
import os
import re
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

DEFAULT_FILE = os.path.join(
    get_package_share_directory("g1_navigation"), "config", "waypoints", "ces_waypoints.yaml"
)


def load_waypoints(path):
    """(name, pose, orientation) in file order; waypoint10 sorts after waypoint9."""
    with open(path, encoding="utf-8") as handle:
        entries = (yaml.safe_load(handle) or {}).get("waypoints") or {}

    def index(name):
        match = re.search(r"(\d+)$", name)
        return int(match.group(1)) if match else 0

    waypoints = []
    for name in sorted(entries, key=index):
        pose, orientation = entries[name]["pose"], entries[name]["orientation"]
        if len(pose) != 3 or len(orientation) != 4:
            raise ValueError(f"{name}: pose needs [x, y, z] and orientation [x, y, z, w]")
        waypoints.append((name, [float(v) for v in pose], [float(v) for v in orientation]))
    if not waypoints:
        raise ValueError(f"{path} has no waypoints")
    return waypoints


class WaypointLoop(Node):
    def __init__(self, frame_id):
        super().__init__("waypoint_loop")
        self._frame_id = frame_id
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal = None

    def wait_for_server(self):
        self.get_logger().info("Waiting for navigate_to_pose (nav:=true)...")
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("...still waiting for navigate_to_pose")

    def go_to(self, pose, orientation):
        """True when the goal succeeds."""
        target = PoseStamped()
        target.header.frame_id = self._frame_id
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = orientation

        sent = self._client.send_goal_async(NavigateToPose.Goal(pose=target))
        rclpy.spin_until_future_complete(self, sent)
        self._goal = sent.result()
        if self._goal is None or not self._goal.accepted:
            self.get_logger().warn("Goal rejected")
            return False

        result = self._goal.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self._goal = None
        return result.result().status == GoalStatus.STATUS_SUCCEEDED

    def cancel(self):
        if self._goal is not None:
            rclpy.spin_until_future_complete(self, self._goal.cancel_goal_async(), timeout_sec=2.0)
            self._goal = None

    def pause(self, seconds):
        if seconds > 0:
            end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
            while rclpy.ok() and self.get_clock().now().nanoseconds < end:
                rclpy.spin_once(self, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?", default=DEFAULT_FILE, help="waypoint YAML")
    parser.add_argument("--loops", type=int, default=0, help="laps to run; 0 runs forever")
    parser.add_argument("--pause", type=float, default=1.0, help="seconds to wait at each pose")
    parser.add_argument("--frame", default="map", help="frame the poses are in")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    waypoints = load_waypoints(args.file)
    rclpy.init()
    node = WaypointLoop(args.frame)
    log = node.get_logger()
    log.info(f"{len(waypoints)} waypoints from {args.file}")

    lap = 0
    try:
        node.wait_for_server()
        while rclpy.ok() and (args.loops == 0 or lap < args.loops):
            lap += 1
            for name, pose, orientation in waypoints:
                log.info(f"Lap {lap}: {name} ({pose[0]:.2f}, {pose[1]:.2f})")
                if node.go_to(pose, orientation):
                    log.info(f"Lap {lap}: reached {name}")
                else:
                    log.warn(f"Lap {lap}: {name} failed, going on to the next one")
                node.pause(args.pause)
        log.info(f"Done after {lap} lap(s)")
    except KeyboardInterrupt:
        log.info("Interrupted, cancelling the goal in flight")
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
