#!/usr/bin/env pythontyp
from acoustic_simulator.packet_class import packet
from acoustic_simulator.soundwave_class import soundwave_cl
import numpy as np
import json
import os
import math
from acoustic_simulator.params import AcousticParams, AnchorParams, AgentParams
from dataclasses import astuple
from rclpy.impl import rcutils_logger
from collections import deque


class modem:

    def __init__(self, acoustic_params: AcousticParams, type, position, id,
                 delay_duration, packet_reception_rate, destination_id,
                 packet_type, init_time):

        self.acoustic_params = acoustic_params

        self.state = "IDLE"

        self.role = type
        self.packet_type = packet_type
        self.position = np.array(astuple(position)).reshape((-1, 1))
        self.modem_id = id
        self.T_wp = self.acoustic_params.t_wp
        self.T_wr = self.acoustic_params.t_wr
        self.poll_trigger = self.acoustic_params.poll_trigger
        self.packet_reception_rate = packet_reception_rate
        self.packet_length_poll = self.acoustic_params.packet_length_poll
        self.packet_length_response = self.acoustic_params.packet_length_response
        self.publish_delay = self.acoustic_params.publish_delay
        self.poll_interval = self.acoustic_params.poll_interval
        self.time_out_alternating = self.acoustic_params.time_out_alternating
        self.number_anchors = self.acoustic_params.number_anchors
        self.algorithm = self.acoustic_params.algorithm

        if self.algorithm == "alternating":
            self.delay_duration = 0.0
        else:
            self.delay_duration = delay_duration

        # das sollte der nächste Zeitpunkt sein, wann delay vorbei ist??
        self.delay_time = init_time

        self.destination_id = destination_id
        self.SOS = self.acoustic_params.sos

        self.sim_time_current = init_time
        self.sim_time_last_step = init_time

        self.soundwave_list = []
        self.last_soundwaveList = []
        self.receiving_list = []
        self.receiving_time_list = []
        self.time_last_poll = init_time
        self.next_poll = self.time_last_poll + self.poll_interval
        self.soundwave_is_in_range = False

        self.transmit_end_time = None
        self.received_soundwave = None
        self.receiving_time = 0

        self.publish_flag = False
        self.published_message = None
        self.last_position = position
        self.transmit_prc_time = 0
        self.anchor_prc_time = self.T_wp + self.delay_duration
        self.ack_counter = 0

        self.poll_permitted = False

        # debugging
        self.sw_counter = 0
        self.realDist = 0

        self.logger = rcutils_logger.RcutilsLogger(name="modem")

    def update(self, position: np.ndarray, soundwave_list: deque,
               sim_time: float, SOS: float, destination_id: str):

        # if len(soundwaveList):
        #     self.logger.info(
        #         f'There are {len(soundwaveList)} soundwaves around!')
        # else:
        #     self.logger.info(f'Currently no travelling soundwaves')

        self.logger.warn(
            f'\n I am a modem of role: {self.role} \nposition shape: {self.position.shape}'
        )

        self.update_destination_id(destination_id)
        self.update_position(position)
        self.update_sim_time(sim_time)
        self.update_soundwave_list(soundwave_list)
        self.update_sos(SOS)
        ret = None

        if self.state == "IDLE":
            if self.algorithm == "broadcast":
                if self.poll_trigger == "time":
                    # T_poll_interval reached
                    if self.role == "agent" and self.next_poll <= self.sim_time_current:
                        self.state = "DELAY"
                        self.resetAckCounter()
                        self.delay_time = self.sim_time_current + self.T_wr

                elif self.poll_trigger == "time_and_last_ack":
                    # T_poll_interval reached
                    if self.role == "agent" and (
                            self.next_poll <= self.sim_time_current
                            or self.ack_counter >= self.number_anchors):
                        self.resetAckCounter()
                        self.state = "DELAY"
                        self.delay_time = self.sim_time_current + self.T_wr

                else:
                    print("[Modem] Wrong Pollcircle!")

            elif self.algorithm == "alternating":
                if self.poll_trigger == "time":
                    if self.role == "agent" and self.next_poll <= self.sim_time_current:
                        self.state = "DELAY"
                        self.delay_time = self.sim_time_current + self.T_wr
                        self.poll_permitted = False

                elif self.poll_trigger == "time_and_last_ack":
                    if self.role == "agent" and (
                            self.poll_permitted
                            or self.next_poll <= self.sim_time_current):
                        self.state = "DELAY"
                        self.delay_time = self.sim_time_current + self.T_wr
                        self.poll_permitted = False

                else:
                    print("[Modem] Wrong Pollcircle!")
            else:
                print("[Modem] Wrong algorithm")

            self.resetReceivingList()

            # sollte das modem nicht nur eine soundwave/packet zur zeit receiven können??
            for soundwave_received in soundwave_list:

                if self.is_in_range(
                        soundwave_received) and self.state == "IDLE":
                    self.received_packet = soundwave_received.get_packet(
                    ).get_packet_dict()
                    self.receiving_time = self.interpolate_receiving_time(
                        soundwave_received)
                    self.receiving_time_list.append(self.receiving_time)
                    self.receiving_list.append(self.received_packet)

            if self.receiving_list:
                index = min(range(len(self.receiving_time_list)),
                            key=self.receiving_time_list.__getitem__)
                self.received_packet = self.receiving_list[index]
                self.receiving_time = self.receiving_time_list[index]
                self.state = "RECEIVE"
                self.exit_time = self.receiving_time + self.received_packet[
                    "length"]

        if self.state == "RECEIVE":
            if self.exit_time <= self.sim_time_current:
                if self.received_packet[
                        "type"] == "TYPE_RANGING_POLL" and self.role == "anchor" and (
                            self.received_packet["dst"] == self.modem_id
                            or self.received_packet["dst"] == "broadcast"):
                    if self.packetLost():
                        self.state = "IDLE"
                    else:
                        self.state = "DELAY"
                        self.delay_time = self.exit_time + self.delay_duration + self.T_wp

                elif self.received_packet[
                        "type"] == "TYPE_RANGING_ACK" and self.role == "agent" and (
                            self.received_packet["dst"] == self.modem_id
                            or self.received_packet["dst"] == "broadcast"):
                    self.runtime = float(
                        self.receiving_time - self.time_last_poll -
                        self.received_packet["AnchorPrcTime"]
                    )  # AnchorPrcTime = delay + PrcTime + Zeitausgleich durch diskrete iteration (schallwelle)

                    self.SOS = self.getSOS()
                    dist = (self.runtime / 2) * self.SOS + float(
                        np.random.normal(
                            self.acoustic_params.meas_noise_mean,
                            self.acoustic_params.meas_noise_std_dev, 1))

                    # Was passiert hier??
                    exittime = self.exit_time + self.publish_delay
                    # Was wird hier abgefragt??
                    if exittime <= self.sim_time_current:
                        if self.packetLost():
                            self.state = "IDLE"
                        else:
                            real_dist = self.real_distance(
                                self.receiving_time,
                                self.received_packet["tx_pos"])
                            dist_error = dist - real_dist  # sollte nur positiv sein
                            self.publish(
                                self.receiving_time,
                                dist,
                                real_dist,
                                dist_error,
                                self.exit_time,
                                self.received_packet["src"],
                                self.received_packet["tx_pos"],
                                self.packet_length_response,
                                self.packet_length_poll,
                            )  # exittime - packetLengthResponse - publishDelay = True meas Time
                            self.poll_permitted = True
                            self.ack_counter += 1
                            self.state = "IDLE"
                else:
                    self.state = "IDLE"

        if self.state == "DELAY":
            if self.delay_time <= self.sim_time_current:
                self.state = "TRANSMIT"
                # set current time, position as transmission time and position
                tx_time = self.sim_time_current
                tx_position = self.position

                if self.role == "agent":
                    packet_to_transmit = packet(
                        config=self.acoustic_params,
                        tx_time=tx_time,
                        tx_pos=tx_position,
                        type=self.packet_type,
                        src=self.modem_id,
                        dst=self.destination_id,
                        time_diff=0.0,  # nicht verwendet, unklar wofür das war
                        length=self.packet_length_poll,
                    )
                    soundwave_to_transmit = soundwave_cl(
                        tx_position, packet_to_transmit)
                    self.transmit_end_time = self.sim_time_current + self.packet_length_poll
                    self.time_last_poll = self.sim_time_current
                    if self.algorithm == "broadcast":
                        self.next_poll = self.time_last_poll + self.poll_interval
                    elif self.algorithm == "alternating":
                        self.next_poll = self.time_last_poll + self.time_out_alternating
                    ret = soundwave_to_transmit
                    self.sw_counter += 1

                if self.role == "anchor":
                    self.transmit_end_time = self.sim_time_current + self.packet_length_response
                    packet_to_transmit = packet(
                        config=self.acoustic_params,
                        tx_time=tx_time,
                        tx_pos=tx_position,
                        type=self.packet_type,
                        src=self.modem_id,
                        dst=self.destination_id,
                        time_diff=0.0,  # nicht verwendet, unklar wofür das war
                        length=self.packet_length_response,
                    )
                    packet_to_transmit.set_anchor_prc_time(
                        self.sim_time_current - self.receiving_time)
                    soundwave_to_transmit = soundwave_cl(
                        tx_position, packet_to_transmit)
                    ret = soundwave_to_transmit
                    self.sw_counter += 1

        if self.state == "TRANSMIT":
            if self.transmit_end_time <= self.sim_time_current:
                self.state = "IDLE"

        return ret  # new Packet for soundwaveList  # Das ist die Soundwave, nicht das packet!!

    def is_in_range(self, soundwave):
        radius_soundwave = soundwave.get_radius_current()
        # if self.position.shape is not (3, 1):
        #     self.logger.warn(f'Position shape not correct!')

        range_receiver = np.linalg.norm(self.position -
                                        soundwave.get_origin_position())

        if radius_soundwave >= range_receiver:
            self.soundwave_is_in_range = True
            return True

        return False

    def interpolate_receiving_time(self, soundwave):
        range_current_time_step = np.absolute(
            np.linalg.norm(
                np.array(self.position -
                         np.array(soundwave.get_origin_position()))))
        # debugging test
        # range_received = np.absolute
        range_last_time_step = np.absolute(
            np.linalg.norm(
                np.array(self.last_position -
                         np.array(soundwave.get_origin_position()))))
        dRangeReceiver = range_current_time_step - range_last_time_step
        dRangeSoundwave = soundwave.get_radius_current(
        ) - soundwave.get_radius_last()

        if self.soundwave_is_in_range == True:  # if soundwave reached receiver and soundwave t-1 has not rechead receiver, calculate intersection with the help of interpolation
            self.t_runtime = self.sim_time_last_step + (
                self.sim_time_current - self.sim_time_last_step
            ) * (soundwave.get_radius_last() - range_last_time_step) / (
                dRangeReceiver - dRangeSoundwave)  # "true" runtime soundwave
            self.sim_time_last_ack = self.sim_time_current
            self.soundwave_is_in_range = False
            return self.t_runtime

    def publish(self, receiving_time, dist, real_dist, dist_error, exit_time,
                ID, position, packet_length_poll, packet_length_response):
        self.publish_flag = True
        self.published_message = {
            "tr": receiving_time,
            "dist": dist,
            "realDist": real_dist,
            "Error": dist_error,
            "time_published": exit_time,
            "ModemID": ID,
            "ModemPos": position,
            "PacketLengthPoll": packet_length_poll,
            "PacketLengthResponse": packet_length_response
        }

    def getSOS(self):
        return self.SOS

    def getPosition(self):
        return self.position

    def resetAckCounter(self):
        self.ack_counter = 0

    def resetReceivingList(self):
        self.receiving_list = []
        self.receiving_time_list = []

    def packetLost(self):
        if np.random.binomial(1, (self.packet_reception_rate)) == 1:
            return False
        return True

    def update_destination_id(self, destination_id):
        self.destination_id = destination_id

    def update_sos(self, SOS):
        self.SOS = SOS

    # Helpfunction
    def real_distance(self, receiving_time, anchor_position):

        agent_position = np.array(self.position)

        realDist = np.linalg.norm(np.array(anchor_position) - agent_position)
        return realDist

    def update_position(self, position):
        self.last_position = self.position
        self.position = position

    def update_sim_time(self, sim_time):
        self.sim_time_last_step = self.sim_time_current
        self.sim_time_current = sim_time

    def update_soundwave_list(self, soundwaveList):
        self.last_soundwaveList = self.soundwave_list
        self.soundwave_list = soundwaveList

    def set_new_measurement_flag(self, bool):
        self.publish_flag = bool

    def send_poll(self, destination_id):
        self.delay_time = self.sim_time_current
        self.update_destination_id(destination_id)
        self.state = "DELAY"

    def new_measurement_available(self):
        return self.publish_flag

    def get_received_measurement(self):
        return self.published_message

    def get_soundwaves(self):
        return self.soundwave_list

    def get_sw_counter(self):
        return self.modem_id, self.sw_counter
