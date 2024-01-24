#!/usr/bin/env python
import numpy as np
from acoustic_simulator.packet_class import packet


class soundwave_cl:

    def __init__(self, origin_position: np.ndarray, packet: packet):
        self.radius_current = 0
        self.radius_last = 0  # at timestep t-1
        self.origin_position = origin_position
        self.packet = packet

    def update(self, dt, SOS):
        self.radius_last = self.radius_current
        self.radius_current += dt * SOS

    def get_radius_current(self):
        return self.radius_current

    def get_radius_last(self):
        return self.radius_last

    def get_origin_position(self):
        return self.origin_position

    def get_packet(self):
        return self.packet

    def set_radius(self, dt, SOS):
        self.radius_current = dt * SOS
        self.radius_last = self.radius_current
