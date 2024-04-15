#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from hippo_msgs.msg import AnchorPoses, AnchorPose
from std_msgs.msg import Float64

# This node publishes ground truth distances between anchor i and agent
# for simpler degugging


class GroundTruthNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.declare_parameter('acoustic_params.number_anchors',
                               rclpy.Parameter.Type.INTEGER)
        self.number_anchors = self.get_parameter(
            'acoustic_params.number_anchors').value

        self.anchor_poses = [None] * self.number_anchors

        self.debug_pub_list = []

        for i in range(self.number_anchors):
            topic_name = '~/debug/anchor_' + str(i + 1) + '/distance'
            pub = self.create_publisher(Float64, topic_name, qos_profile=1)
            self.debug_pub_list.append(pub)

        self.odometry_sub = self.create_subscription(Odometry,
                                                     'ground_truth/odometry',
                                                     self.on_odometry,
                                                     qos_profile=1)
        self.anchor_poses_sub = self.create_subscription(AnchorPoses,
                                                         'anchor_poses',
                                                         self.on_anchor_poses,
                                                         qos_profile=1)

    def on_odometry(self, msg: Odometry):
        p = msg.pose.pose
        anchor: AnchorPose
        for anchor in self.anchor_poses:
            if anchor is None:
                return
            msg_out = Float64()
            msg_out.data = self.get_dist(p, anchor.pose.pose)
            self.debug_pub_list[anchor.id - 1].publish(msg_out)

    def on_anchor_poses(self, msg: AnchorPoses):
        anchor: AnchorPose
        for i, anchor in enumerate(msg.anchors):
            self.anchor_poses[
                anchor.id -
                1] = anchor  # todo: fix index mess - start anchors at 0 with modem id 1? or give agent modem id 99

    def get_dist(self, pose0: Pose, pose1: Pose) -> float:
        return math.sqrt((pose0.position.x - pose1.position.x)**2 +
                         (pose0.position.y - pose1.position.y)**2 +
                         (pose0.position.z - pose1.position.z)**2)


def main():
    rclpy.init()
    node = GroundTruthNode("acoustic_distances_ground_truth")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
