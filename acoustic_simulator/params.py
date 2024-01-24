#!/usr/bin/env python
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from dataclasses import astuple, dataclass, is_dataclass
from typing import Any


def read_params_recursion(data: Any, name: str, node: Node):
    if not is_dataclass(data):
        descriptor = ParameterDescriptor()
        if isinstance(data, int):
            descriptor.type = ParameterType.PARAMETER_INTEGER
        elif isinstance(data, float):
            descriptor.type = ParameterType.PARAMETER_DOUBLE
        elif isinstance(data, str):
            descriptor.type = ParameterType.PARAMETER_STRING
        else:
            raise TypeError(f'Unsupported type "{type(data)}"')
        node.declare_parameter(name, descriptor=descriptor)
        return node.get_parameter(name).value
    for field_name in data.__annotations__.keys():
        setattr(
            data, field_name,
            read_params_recursion(getattr(data, f'{field_name}'),
                                  f'{name}.{field_name}', node))
    return data


@dataclass()
class ModemParams:
    id: int = 0
    delay_time: float = 0.0
    packet_reception_rate: float = 0.0
    destination_id: str = ''
    packet_type: str = ''


@dataclass
class PositionParams:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass()
class AnchorParams:
    modem: ModemParams
    position: PositionParams
    name: str = ''


# todo: find better solution - merge with AnchorParams?
@dataclass()
class AgentParams:
    modem: ModemParams
    position: PositionParams
    name: str = ''


@dataclass
class AcousticParams:
    type: str = ''
    packet_length_poll: float = 0.0
    packet_length_response: float = 0.0
    sos: float = 0.0
    algorithm: str = ''
    time_out_alternating: float = 0.0
    poll_trigger: str = ''
    poll_interval: float = 0.0
    t_wp: float = 0.0
    t_wr: float = 0.0
    publish_delay: float = 0.0
    time_out: float = 0.0
    number_anchors: int = 0
    frequency_acoustic_sim: float = 0.0
    meas_noise_mean: float = 0.0
    meas_noise_std_dev: float = 0.0
