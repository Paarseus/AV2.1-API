"""
Actuator Interfaces

Hardware interfaces for vehicle control via Teensy CAN Master.

Classes:
    VehicleActuator: Serial/USB connection
    VehicleActuatorUDP: UDP/Ethernet connection
"""

from .serial import VehicleActuator
from .udp import VehicleActuatorUDP

__all__ = ["VehicleActuator", "VehicleActuatorUDP"]
