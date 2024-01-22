#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from hippo_msgs.msg import ModemOut, AnchorPoses, AnchorPose
from visualization_msgs.msg import Marker, MarkerArray
from acoustic_simulator.acoustic_sim_class import acousticSimulation
from acoustic_simulator.params import read_params_recursion, AnchorParams, AcousticParams, AgentParams, ModemParams, PositionParams

import threading


class SimulateAnchorMeasurementsNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.declare_parameter('anchors', rclpy.Parameter.Type.STRING_ARRAY)
        self.anchors = self.init_anchor_params(
            self.get_parameter('anchors').value)
        self.number_anchors = len(self.anchors)
        self.acoustic_params = self.init_acoustic_params()
        self.agent = self.init_agent_params()

        self.lock = threading.RLock()
        self.agent_position = [0.0, 0.0,
                               0.0]  # todo: ideally, we want hydrophon pose
        self.agent_velocity = [0.0, 0.0, 0.0]

        self.rate_accoustics = 100.0  # Hz

        init_time = self.get_clock().now()
        init_time = init_time.nanoseconds * 1e-9
        self.acoustic_sim = acousticSimulation(self.acoustic_params, self.agent,
                                               self.anchors, init_time)

        self.modem_pub = self.create_publisher(ModemOut,
                                               'modems',
                                               qos_profile=1)

        self.anchor_poses_pub = self.create_publisher(AnchorPoses,
                                                      'anchor_poses',
                                                      qos_profile=1)

        self.modem_publisher_list = []
        for i in range(self.number_anchors):
            topic_name = 'modem_' + str(
                i + 1)  # TODO: fix modem ids starting at 1
            pub = self.create_publisher(ModemOut, topic_name, qos_profile=1)
            self.modem_publisher_list.append(pub)

        self.modem_error_publisher_list = []
        for i in range(self.number_anchors):
            topic_name = 'modem_' + str(i + 1) + '/error'
            pub = self.create_publisher(ModemOut, topic_name, qos_profile=1)
            self.modem_error_publisher_list.append(pub)

        self.marker_pub = self.create_publisher(MarkerArray,
                                                '~/marker_array',
                                                qos_profile=1)

        self.odometry_sub = self.create_subscription(Odometry,
                                                     'odometry',
                                                     self.on_odometry,
                                                     qos_profile=1)

        self.acoustics_timer = self.create_timer(
            timer_period_sec=(1 / self.rate_accoustics),
            callback=self.simulate_acoustics)

        self.anchor_poses_timer = self.create_timer(
            timer_period_sec=(1 / 1.0), callback=self.publish_anchor_poses)

    def simulate_acoustics(self):
        t = self.get_clock().now()
        t = t.nanoseconds * 1e-9  # convert time to nanoseconds
        with self.lock:
            # What about moving buoys etc?
            measurement = self.acoustic_sim.simulate(self.agent_position, t)

            if measurement is None:
                return

            self.send_measurement(measurement["ModemID"], measurement["dist"],
                                  measurement["time_published"])
            # self.get_logger().info(
            #     f'Received measurement from modem {measurement["ModemID"]}, distance measured: {measurement["dist"]:.2f} \n Agent position was: {self.agent_position[0]}, {self.agent_position[1]}, {self.agent_position[2]}'
            # )

        # publish anchor rviz markers
        self.publish_rviz_anchors(self.anchors)

    def on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.agent_position = [p.x, p.y, p.z]
            self.agent_velocity = [v.x, v.y, v.z]

    def send_measurement(self, id: int, dist: float, t: float):
        msg = ModemOut()
        msg.header = self.fill_header_timestamp(t)
        msg.distance = dist
        msg.id = id

        # publish all measurements in same topic
        self.modem_pub.publish(msg)

        # publish measurement in modem-specific topic
        self.modem_publisher_list[id - 1].publish(msg)

    def send_distance_error(self, id: int, err: float, t: float):
        # TODO: add 'error' entry to ModemOut instead
        msg = ModemOut()
        msg.header = self.fill_header_timestamp(t)
        msg.distance = err
        msg.id = id

        self.modem_error_publisher_list[id - 1].publish(msg)

    def fill_header_timestamp(self, t: float) -> Header:
        # convert float seconds to int seconds + int nanoseconds
        # to be compatible with time stamp msg type
        header = Header()
        header.stamp.sec = int(t)
        header.stamp.nanosec = int((t - int(t)) * 1e9)
        return header

    def init_acoustic_params(self) -> AcousticParams:
        acoustic_params = read_params_recursion(AcousticParams(),
                                                'acoustic_params', self)
        return acoustic_params

    def init_agent_params(self):
        agent_params = read_params_recursion(
            AgentParams(modem=ModemParams(), position=PositionParams()),
            'agent', self)
        return agent_params

    def init_anchor_params(self, anchor_names: list[str]) -> list[AnchorParams]:
        anchors = []
        for name in anchor_names:
            anchor_params = read_params_recursion(
                AnchorParams(modem=ModemParams(), position=PositionParams()),
                name, self)
            anchors.append(anchor_params)
        return anchors

    def publish_rviz_markers(self, markers):
        self.marker_pub.publish(MarkerArray(markers=markers))

    def publish_rviz_anchors(self, anchors: list[AnchorParams]):
        markers = []
        for anchor in anchors:
            marker, text_marker = self.create_anchor_marker(anchor)
            markers.append(marker)
            markers.append(text_marker)
        self.publish_rviz_markers(markers)

    def publish_anchor_poses(self):
        msg = AnchorPoses()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        for anchor in self.anchors:
            single_anchor_msg = AnchorPose()
            single_anchor_msg.pose.header.stamp = now.to_msg()
            single_anchor_msg.pose.header.frame_id = 'map'
            single_anchor_msg.id = anchor.modem.id
            single_anchor_msg.pose.pose.position.x = anchor.position.x
            single_anchor_msg.pose.pose.position.y = anchor.position.y
            single_anchor_msg.pose.pose.position.z = anchor.position.z
            msg.anchors.append(single_anchor_msg)
        self.anchor_poses_pub.publish(msg)

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


def main():
    rclpy.init()
    node = SimulateAnchorMeasurementsNode("acoustic_sim_test_node")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
