"""Hand coupler: 6-actuator commands -> 12-joint finger trajectories.

The Inspire RH56DFQ hand has 6 linear actuators; each finger's distal joints are
driven by a fixed linkage. Gazebo's DART engine cannot enforce URDF <mimic>
constraints, so this node expands each 6-value command into linkage-consistent
targets for every joint (the same mapping the real hand's linkage performs).

Subscribes  /loco/{left,right}_hand/command   std_msgs/Float64MultiArray
            data = [thumb_yaw, thumb_pitch, index, middle, ring, pinky]  (rad)
Publishes   /{side}_hand_controller/joint_trajectory  trajectory_msgs/JointTrajectory
Parameter   move_time (s, default 0.5)
"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTUATORS = ["thumb_proximal_yaw", "thumb_proximal_pitch", "index_proximal", "middle_proximal",
             "ring_proximal", "pinky_proximal"]


class HandCoupler(Node):
    def __init__(self):
        super().__init__("hand_coupler")
        self.declare_parameter("move_time", 0.5)
        path = os.path.join(get_package_share_directory("midibot_description"), "config", "hand_coupling.json")
        with open(path) as f:
            self.coupling = json.load(f)
        self.pubs = {}
        for side, prefix in (("left", "L"), ("right", "R")):
            self.pubs[side] = self.create_publisher(JointTrajectory, f"/{side}_hand_controller/joint_trajectory", 10)
            self.create_subscription(Float64MultiArray, f"/loco/{side}_hand/command",
                                     lambda m, s=side, p=prefix: self.on_cmd(s, p, m), 10)
        self.get_logger().info("hand coupler ready: /loco/{left,right}_hand/command (6 values)")

    def on_cmd(self, side, prefix, msg):
        if len(msg.data) != 6:
            self.get_logger().error(f"{side} hand command needs 6 values, got {len(msg.data)}")
            return
        names, pos = [], []
        table = self.coupling.get(f"{side}_hand", {})
        for act, q in zip(ACTUATORS, msg.data):
            j = f"{prefix}_{act}_joint"
            names.append(j)
            pos.append(q)
            for dep in table.get(j, []):
                names.append(dep["joint"])
                pos.append(q * dep["multiplier"] + dep["offset"])
        t = float(self.get_parameter("move_time").value)
        traj = JointTrajectory(joint_names=names)
        traj.points = [JointTrajectoryPoint(positions=pos, time_from_start=Duration(
            sec=int(t), nanosec=int((t % 1) * 1e9)))]
        self.pubs[side].publish(traj)


def main():
    rclpy.init()
    rclpy.spin(HandCoupler())


if __name__ == "__main__":
    main()
