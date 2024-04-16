#!/usr/bin/env python
import numpy as np
from acoustic_simulator.params import AcousticParams


class packet:

    def __init__(self, config: AcousticParams, tx_time: float,
                 tx_pos: np.ndarray, type: str, src: str, dst: str,
                 length: float):
        self.acoustic_params = config
        self.tx_time = tx_time  # Erstellungszeit
        self.tx_pos = tx_pos  # Erstellungsort
        self.length = length  # Länge des Pakets [s]
        self.anchor_prc_time = 0  # Verarbeitungszeit des Pakets, hier Annahme Null
        self.type = type  # Poll oder Response
        self.src = src  # Quell-ID
        self.dst = dst  # Ziel-ID
        self.timeout = self.acoustic_params.time_out + self.tx_time  # timeout aus configfile

    def get_packet_dict(self):
        self.dict = {
            "tx_time": self.tx_time,
            "tx_pos": self.tx_pos,
            "type": self.type,
            "src": self.src,
            "dst": self.dst,
            "timeout": self.timeout,
            "length": self.length,
            "AnchorPrcTime": self.anchor_prc_time
        }
        return self.dict

    def set_anchor_prc_time(self, t):
        self.anchor_prc_time = t
