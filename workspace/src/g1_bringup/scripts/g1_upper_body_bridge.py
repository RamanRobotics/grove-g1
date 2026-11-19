#!/usr/bin/env python3
"""
upper_body_slider_gui.py
Slider GUI for g1_upper_body_bridge.py: one slider per upper-body joint, limited to
the same joint limits (minus the same margin) the bridge uses.

  Subscribes: /upper_body/joint_states   (measured pose, used to initialise sliders)
  Publishes:  /upper_body/joint_command  (only joints you have moved)
  Calls:      /g1_upper_body_bridge/engage, /g1_upper_body_bridge/release

Only needs sensor_msgs and std_srvs, so it can run on your laptop (ROS 2 on the
same network) or on the robot over `ssh -Y`.

Usage:
  sudo apt install python3-tk          # once, if tkinter is missing
  python3 upper_body_slider_gui.py --ros-args -p dof:=23

Safety:
  - Sliders start at the MEASURED pose; nothing is sent until you move a slider.
  - Only joints you touched are sent, so other joints keep their current targets.
  - "Live" off (default): move sliders, then press Send.
    "Live" on: changes are sent continuously while you drag.
  - "Sync to robot" resets all sliders to the measured pose and forgets changes.
  - The bridge still rate-limits and clamps every command.
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# Same limits [rad] and joint sets as g1_upper_body_bridge.py
JOINT_LIMITS = {
    "waist_yaw": (-2.618, 2.618),
    "waist_roll": (-0.52, 0.52),
    "waist_pitch": (-0.52, 0.52),
    "left_shoulder_pitch": (-3.0892, 2.6704),
    "left_shoulder_roll": (-1.5882, 2.2515),
    "left_shoulder_yaw": (-2.618, 2.618),
    "left_elbow": (-1.0472, 2.0944),
    "left_wrist_roll": (-1.972, 1.972),
    "left_wrist_pitch": (-1.614, 1.614),
    "left_wrist_yaw": (-1.614, 1.614),
    "right_shoulder_pitch": (-3.0892, 2.6704),
    "right_shoulder_roll": (-2.2515, 1.5882),
    "right_shoulder_yaw": (-2.618, 2.618),
    "right_elbow": (-1.0472, 2.0944),
    "right_wrist_roll": (-1.972, 1.972),
    "right_wrist_pitch": (-1.614, 1.614),
    "right_wrist_yaw": (-1.614, 1.614),
}
JOINTS_29DOF = list(JOINT_LIMITS)
JOINTS_23DOF = [j for j in JOINTS_29DOF if j not in (
    "waist_roll", "waist_pitch", "left_wrist_pitch", "left_wrist_yaw",
    "right_wrist_pitch", "right_wrist_yaw")]


class SliderNode(Node):
    def __init__(self):
        super().__init__("upper_body_slider_gui")
        self.declare_parameter("dof", 23)
        self.declare_parameter("limit_margin", 0.1)
        self.declare_parameter("bridge_name", "g1_upper_body_bridge")
        self.declare_parameter("live_rate_hz", 10.0)

        dof = self.get_parameter("dof").value
        self.joints = JOINTS_29DOF if dof == 29 else JOINTS_23DOF
        margin = self.get_parameter("limit_margin").value
        self.limits = {j: (JOINT_LIMITS[j][0] + margin, JOINT_LIMITS[j][1] - margin)
                       for j in self.joints}
        self.live_period_ms = int(1000 / self.get_parameter("live_rate_hz").value)

        self.measured = {}
        self.lock = threading.Lock()
        self.create_subscription(JointState, "upper_body/joint_states", self.on_state, 10)
        self.pub = self.create_publisher(JointState, "upper_body/joint_command", 10)

        bridge = self.get_parameter("bridge_name").value
        self.engage_cli = self.create_client(Trigger, f"/{bridge}/engage")
        self.release_cli = self.create_client(Trigger, f"/{bridge}/release")

    def on_state(self, msg):
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.measured[n] = p

    def get_measured(self):
        with self.lock:
            return dict(self.measured)

    def send(self, targets):
        if not targets:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(targets)
        js.position = [float(v) for v in targets.values()]
        self.pub.publish(js)

    def call(self, client, on_done):
        if not client.service_is_ready():
            on_done(False, "Bridge service not available - is the bridge running?")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: on_done(f.result().success, f.result().message)
            if f.result() else on_done(False, "Service call failed"))


class SliderGUI:
    def __init__(self, node: SliderNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("G1 upper body - joint command")
        self.vars, self.meas_labels = {}, {}
        self.dirty = set()
        self.initialised = False
        self.updating = False  # True while sliders are set programmatically

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Button(top, text="Engage", command=self.engage).pack(side="left", padx=2)
        ttk.Button(top, text="Release", command=self.release).pack(side="left", padx=2)
        ttk.Button(top, text="Sync to robot", command=self.sync).pack(side="left", padx=2)
        ttk.Button(top, text="Send", command=self.send_dirty).pack(side="left", padx=2)
        self.live = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Live", variable=self.live).pack(side="left", padx=8)

        grid = ttk.Frame(self.root, padding=8)
        grid.pack(fill="both", expand=True)
        ttk.Label(grid, text="Joint").grid(row=0, column=0, sticky="w")
        ttk.Label(grid, text="Target [rad]").grid(row=0, column=1)
        ttk.Label(grid, text="Measured").grid(row=0, column=2)
        for r, j in enumerate(node.joints, start=1):
            lo, hi = node.limits[j]
            ttk.Label(grid, text=j).grid(row=r, column=0, sticky="w", padx=(0, 8))
            var = tk.DoubleVar(value=0.0)
            scale = tk.Scale(grid, variable=var, from_=lo, to=hi, resolution=0.01,
                             orient="horizontal", length=320, state="disabled",
                             command=lambda _v, name=j: self.on_slide(name))
            scale.grid(row=r, column=1, sticky="ew")
            meas = ttk.Label(grid, text="--", width=8)
            meas.grid(row=r, column=2)
            self.vars[j] = (var, scale)
            self.meas_labels[j] = meas

        self.status = tk.StringVar(value="Waiting for /upper_body/joint_states ...")
        ttk.Label(self.root, textvariable=self.status, padding=8).pack(fill="x")

        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self.root.after(100, self.refresh)
        self.root.after(self.node.live_period_ms, self.live_tick)

    # ----- actions -----
    def on_slide(self, name):
        if not self.updating:
            self.dirty.add(name)

    def sync(self):
        meas = self.node.get_measured()
        if not all(j in meas for j in self.node.joints):
            self.status.set("No measured pose yet")
            return
        self.updating = True
        for j, (var, scale) in self.vars.items():
            scale.configure(state="normal")
            var.set(round(meas[j], 2))
        self.updating = False
        self.dirty.clear()
        self.status.set("Sliders synced to measured pose")

    def send_dirty(self):
        targets = {j: self.vars[j][0].get() for j in self.dirty}
        self.node.send(targets)
        if targets and not self.live.get():
            self.status.set(f"Sent: {', '.join(targets)}")

    def engage(self):
        self.sync()  # sliders match the pose the bridge will hold
        self.node.call(self.node.engage_cli, self.show_result)

    def release(self):
        self.node.call(self.node.release_cli, self.show_result)

    def show_result(self, ok, message):
        self.root.after(0, lambda: self.status.set(("OK: " if ok else "ERROR: ") + message))

    # ----- periodic -----
    def refresh(self):
        meas = self.node.get_measured()
        for j, label in self.meas_labels.items():
            if j in meas:
                label.configure(text=f"{meas[j]:+.2f}")
        if not self.initialised and all(j in meas for j in self.node.joints):
            self.sync()
            self.initialised = True
        self.root.after(100, self.refresh)

    def live_tick(self):
        if self.live.get() and self.dirty:
            self.send_dirty()
        self.root.after(self.node.live_period_ms, self.live_tick)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = SliderNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        SliderGUI(node).run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
