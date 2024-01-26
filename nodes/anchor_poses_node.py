#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from hippo_msgs.msg import AnchorPoses, AnchorPose
from visualization_msgs.msg import MarkerArray, Marker
from acoustic_simulator.params import read_params_recursion, AnchorParams, ModemParams, PositionParams


class AnchorPosesNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.declare_parameter('anchors', rclpy.Parameter.Type.STRING_ARRAY)
        self.anchors = self.init_anchor_params(
            self.get_parameter('anchors').value)
        self.number_anchors = len(self.anchors)

        self.anchor_poses_pub = self.create_publisher(AnchorPoses,
                                                      'anchor_poses',
                                                      qos_profile=1)

        self.marker_pub = self.create_publisher(MarkerArray,
                                                '~/marker_array',
                                                qos_profile=1)
        self.anchor_poses_timer = self.create_timer(
            timer_period_sec=(1 / 30.0), callback=self.on_anchor_timer)

    def on_anchor_timer(self):
        self.publish_anchor_poses(self.anchors)
        self.publish_rviz_anchors(self.anchors)

    def publish_anchor_poses(self, anchors: list[AnchorParams]):
        msg = AnchorPoses()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        for anchor in anchors:
            single_anchor_msg = AnchorPose()
            single_anchor_msg.pose.header.stamp = now.to_msg()
            single_anchor_msg.pose.header.frame_id = 'map'
            single_anchor_msg.id = anchor.modem.id
            single_anchor_msg.pose.pose.position.x = anchor.position.x
            single_anchor_msg.pose.pose.position.y = anchor.position.y
            single_anchor_msg.pose.pose.position.z = anchor.position.z
            msg.anchors.append(single_anchor_msg)
        self.anchor_poses_pub.publish(msg)

    def publish_rviz_anchors(self, anchors: list[AnchorParams]):
        markers = []
        for anchor in anchors:
            marker, text_marker = self.create_anchor_marker(anchor)
            markers.append(marker)
            markers.append(text_marker)
        self.publish_rviz_markers(markers)

    def create_anchor_marker(self, anchor: AnchorParams) -> Marker:
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.id = anchor.modem.id
        marker.pose = Pose()
        marker.pose.position.x = anchor.position.x
        marker.pose.position.y = anchor.position.y
        marker.pose.position.z = anchor.position.z
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.header.frame_id = 'map'
        marker.ns = 'anchors'

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.id = self.number_anchors + anchor.modem.id
        text_marker.pose = Pose()
        text_marker.pose.position.x = anchor.position.x
        text_marker.pose.position.y = anchor.position.y
        text_marker.pose.position.z = anchor.position.z + 0.5
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.b = 1.0
        text_marker.color.g = 1.0
        text_marker.scale.z = 0.5  # only scale for text markers
        text_marker.text = f"{anchor.name}"
        text_marker.header.frame_id = 'map'
        text_marker.ns = 'anchors'
        return marker, text_marker

    def publish_rviz_markers(self, markers):
        self.marker_pub.publish(MarkerArray(markers=markers))

    def init_anchor_params(self, anchor_names: list[str]) -> list[AnchorParams]:
        anchors = []
        for name in anchor_names:
            anchor_params = read_params_recursion(
                AnchorParams(modem=ModemParams(), position=PositionParams()),
                name, self)
            anchors.append(anchor_params)
        return anchors


def main():
    rclpy.init()
    node = AnchorPosesNode("anchor_poses")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
