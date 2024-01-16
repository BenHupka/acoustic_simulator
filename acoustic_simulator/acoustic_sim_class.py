#!/usr/bin/env python
import rclpy
from rclpy.impl import rcutils_logger
import numpy as np
from collections import deque
from acoustic_simulator.modem_class import modem
from acoustic_simulator.params import AnchorParams, AcousticParams, AgentParams
from typing import Any


class acousticSimulation:

    def __init__(self, acoustic_params: AcousticParams, agent: AgentParams,
                 anchors: list[AnchorParams], init_time: float):
        self.acoustic_params = acoustic_params
        self.anchors = anchors
        self.agent = agent
        self.logger = rcutils_logger.RcutilsLogger(name="acoustic_sim")

        self.t = init_time
        self.last_t = init_time
        self.dt = None

        self.soundwave_list = deque([])

        self.agent_modem_list = self.fill_agent_modem_list()
        self.anchor_modem_list = self.fill_anchor_modem_list()

        if self.acoustic_params.algorithm == "broadcast":
            self.AgentDst = "broadcast"
            self.AnchorDst = "broadcast"
        elif self.acoustic_params.algorithm == "alternating":
            self.AgentDst = self.anchor_id_list[0]
            self.AnchorDst = self.agent_id_list[0]
        else:
            print("[Acousitc-sim] Dst Error")

        self.agent_modem_list[0].sendPoll(
            self.AgentDst)  # Agent sends first Poll
        self.dst_counter = 0

    def simulate(self, agent_pose: np.ndarray, t: float) -> Any:
        self.t = float(t)
        self.dt = float(self.t - self.last_t)
        self.last_t = self.t

        self.update_soundwave(self.dt)
        self.delete_soundwave()
        self.update_agent_modem(agent_pose)
        self.update_anchor_modem()
        return self.get_published_modem_measurement(
        )  # measurement, is what type?!

    def update_soundwave(self, dt: float):
        sos = self.acoustic_params.sos
        for soundwave in self.soundwave_list:
            soundwave.update(dt, sos)

    def update_agent_modem(self, x):
        for modem in self.agent_modem_list:
            ret = modem.update(x, self.soundwave_list, self.t,
                               self.acoustic_params.sos, self.AgentDst)
            if ret is not None:
                self.soundwave_list.append(ret)

    def update_anchor_modem(self):
        for modem in self.anchor_modem_list:
            ret = modem.update(modem.getPosition(), self.soundwave_list, self.t,
                               self.acoustic_params.sos, self.AnchorDst)
            if ret is not None:
                self.soundwave_list.append(ret)

    def get_published_modem_measurement(self):
        if self.dst_counter <= self.t:
            self.new_dst(self.AgentDst)

        meas = None
        for modem in self.agent_modem_list:
            if modem.getPublished():
                meas = modem.getPublishedMessage()
                modem.setPublishedFlag(False)
                self.new_dst(meas["ModemID"])
                self.set_dst_counter()
                # self.logger.info(f'Measurement received: {meas}')
        return meas

    def delete_soundwave(self):
        counter = 0
        for soundwave in self.soundwave_list:
            if soundwave.getPacket().getPacketDict()["timeout"] <= self.t:
                counter += 1

        for i in range(counter):
            self.soundwave_list.popleft()

    def set_dst_counter(self):  # Was passiert hier?
        self.dst_counter = self.t + self.acoustic_params.time_out_alternating

    def new_dst(self, ID):
        if self.acoustic_params.algorithm == "broadcast":
            self.AgentDst = "broadcast"
            self.AnchorDst = "broadcast"
        elif self.acoustic_params.algorithm == "alternating":
            for i in range(len(self.anchor_id_list)):
                if self.anchor_id_list[i] == ID:
                    if i < len(self.anchor_id_list) - 1:
                        self.AgentDst = self.anchor_id_list[i + 1]
                    elif i >= len(self.anchor_id_list) - 1:
                        self.AgentDst = self.anchor_id_list[0]

    def fill_anchor_modem_list(self):
        anchor_list = []
        self.anchor_id_list = []

        for anchor in self.anchors:
            anchor_modem = modem(
                self.acoustic_params,
                "anchor",  # type/role - bisher noch benötigt
                anchor.position,
                anchor.modem.id,
                anchor.modem.delay_time,
                anchor.modem.packet_reception_rate,
                anchor.modem.dst,
                anchor.modem.packet_type)
            anchor_list.append(anchor_modem)
            self.anchor_id_list.append(anchor.modem.id)
            # self.logger.info(f'Adding modem with id {anchor.modem.id}')

        return anchor_list

    def fill_agent_modem_list(self):
        agent_list = []
        self.agent_id_list = []

        # refactored, only one agent for now:
        agent_modem = modem(
            self.acoustic_params,
            "agent",  # type/role - bisher noch benötigt
            self.agent.position,
            self.agent.modem.id,
            self.agent.modem.delay_time,
            self.agent.modem.packet_reception_rate,
            self.agent.modem.dst,
            self.agent.modem.packet_type)
        agent_list.append(agent_modem)
        self.agent_id_list.append(self.agent.modem.id)
        return agent_list


def main():
    sim = acousticSimulation()


if __name__ == "__main__":
    main()
