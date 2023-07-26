#!/usr/bin/env python
class soundwave_cl:

    def __init__(self, position, packet):
        self.radius = 0
        self.last_radius = 0   # at timestep t-1
        self.position = position
        self.packet = packet

    def update(self, dt, SOS):
        self.last_radius = self.radius
        self.radius += dt * SOS

    def get_radius(self):
        return self.radius

    def get_last_radius(self):
        return self.last_radius

    def get_position(self):
        return self.position

    def get_packet(self):
        return self.packet

    def set_radius(self, dt, SOS):
        self.radius = dt * SOS
        self.last_radius = self.radius
