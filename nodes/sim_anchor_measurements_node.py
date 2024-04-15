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
import numpy as np

# hippo_simulation/hippo_sim/models/bluerov/urdf/bluerov_macro.xacro:
#  - ground truth odometry publish rate auf 300 setzen (simulation läuft mit 250Hz)


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
        init_time = init_time.nanoseconds * 1e-9  # to seconds
        self.acoustic_sim = acousticSimulation(self.acoustic_params, self.agent,
                                               self.anchors, init_time)

        self.modem_pub = self.create_publisher(ModemOut,
                                               'modems',
                                               qos_profile=1)

        self.modem_publisher_list = []
        for i in range(self.number_anchors):
            topic_name = 'modem_' + str(
                i + 1)  # TODO? - fix index mess, modems starting at 1
            pub = self.create_publisher(ModemOut, topic_name, qos_profile=1)
            self.modem_publisher_list.append(pub)

        self.modem_error_publisher_list = []
        for i in range(self.number_anchors):
            topic_name = 'modem_' + str(i + 1) + '/error'
            pub = self.create_publisher(ModemOut, topic_name, qos_profile=1)
            self.modem_error_publisher_list.append(pub)

        self.odometry_sub = self.create_subscription(Odometry,
                                                     'ground_truth/odometry',
                                                     self.on_odometry,
                                                     qos_profile=1)

        self.anchor_poses_sub = self.create_subscription(AnchorPoses,
                                                         'anchor_poses',
                                                         self.on_anchor_poses,
                                                         qos_profile=1)

        self.acoustics_timer = self.create_timer(
            timer_period_sec=(1 / self.rate_accoustics),
            callback=self.simulate_acoustics)

    def simulate_acoustics(self):
        t = self.get_clock().now().nanoseconds
        t = t * 1e-9  # convert time to seconds
        with self.lock:
            measurement = self.acoustic_sim.simulate(self.agent_position,
                                                     self.anchors, t)

            if measurement is None:
                return

            self.send_measurement(measurement["ModemID"], measurement["dist"],
                                  measurement["time_published"])

            self.send_distance_error(measurement["ModemID"],
                                     measurement["Error"],
                                     measurement["time_published"])
            self.get_logger().info(
                f'At time t={t} \n Received measurement from modem {measurement["ModemID"]}, distance measured: {measurement["dist"]:.2f}'
            )

    def on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.agent_position = np.array([p.x, p.y, p.z]).reshape((3, 1))
            self.agent_velocity = np.array([v.x, v.y, v.z]).reshape((3, 1))

    def on_anchor_poses(self, msg: AnchorPoses):
        # update positions of anchors to potentially consider drifting drones
        for anchor_msg in msg.anchors:
            # find matching ids
            self.anchors[anchor_msg.id -
                         1].position.x = anchor_msg.pose.pose.position.x
            self.anchors[anchor_msg.id -
                         1].position.y = anchor_msg.pose.pose.position.y
            self.anchors[anchor_msg.id -
                         1].position.z = anchor_msg.pose.pose.position.z

    def send_measurement(self, id: int, dist: float, t: float):
        msg = ModemOut()
        msg.header = self.fill_header_timestamp(t)
        msg.distance = dist
        msg.id = id

        # publish all measurements in same topic
        self.modem_pub.publish(msg)

        # publish measurement in modem-specific topic
        self.modem_publisher_list[id - 1].publish(msg)

    def send_distance_error(self, id: int, error: float, t: float):
        # TODO: add 'error' entry to ModemOut instead?
        msg = ModemOut()
        msg.header = self.fill_header_timestamp(t)
        msg.distance = error
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


def main():
    rclpy.init()
    node = SimulateAnchorMeasurementsNode("acoustic_sim_test_node")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
