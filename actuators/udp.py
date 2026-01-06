#!/usr/bin/env python3
"""
Vehicle Actuator Interface via UDP/Ethernet

Communicates with Teensy 4.1 CAN Master over Ethernet.

Protocol:
    Commands:
        E 1|0      -> estop on/off
        T 0..1     -> throttle
        M N|D|S|R  -> mode (Neutral/Drive/Sport/Reverse)
        B 0..1     -> brake
        S -1..1    -> steer (left -, right +)
        C          -> center steering
        A E=0 T=0.5 M=D B=0 S=0.1  -> all-in-one
        P          -> request state (returns JSON)

    Response (JSON):
        {"e":0,"t":0.500,"m":"D","b":0.000,"s":0.100,"w":1}

Network:
    Default IP: 192.168.13.177
    Default Port: 5005
    Watchdog: 500ms (Teensy triggers E-stop if no command received)
"""

from __future__ import annotations
import socket
import threading
import time
import json
from typing import Optional, Dict, Any
from dataclasses import dataclass

VALID_MODES = ("N", "D", "S", "R")
DEFAULT_IP = "192.168.13.177"
DEFAULT_PORT = 5005


@dataclass
class ActuatorState:
    """Current actuator state from Teensy."""
    estop: bool = False
    throttle: float = 0.0
    mode: str = "N"
    brake: float = 0.0
    steer: float = 0.0
    watchdog_active: bool = False
    timestamp: float = 0.0


class VehicleActuatorUDP:
    """Vehicle actuator interface via UDP/Ethernet."""

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 timeout: float = 0.1, keepalive: bool = True,
                 keepalive_interval: float = 0.2):
        """
        Args:
            ip: Teensy IP address
            port: UDP port
            timeout: Socket receive timeout (seconds)
            keepalive: Send periodic commands to prevent watchdog timeout
            keepalive_interval: Seconds between keepalive packets (< 0.5s)
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.keepalive_enabled = keepalive
        self.keepalive_interval = keepalive_interval

        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._state = ActuatorState()
        self._last_send_time = time.time()  # Initialize to current time to prevent immediate keepalive

        # Keepalive thread
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_stop = threading.Event()

        # Current values (for keepalive resend)
        self._throttle = 0.0
        self._brake = 0.0
        self._steer = 0.0
        self._mode = "N"
        self._estop = False

        self.open()

    def open(self):
        """Open UDP socket and start keepalive."""
        if self._socket:
            return

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(self.timeout)
        self._socket.bind(("", 0))

        print(f"[ACTUATOR-UDP] Connected to {self.ip}:{self.port}")

        if self.keepalive_enabled:
            self._keepalive_stop.clear()
            self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
            self._keepalive_thread.start()

    def close(self):
        """Close UDP socket and stop keepalive."""
        if self._keepalive_thread:
            self._keepalive_stop.set()
            self._keepalive_thread.join(timeout=1.0)
            self._keepalive_thread = None

        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            print("[ACTUATOR-UDP] Closed")

    def __enter__(self) -> "VehicleActuatorUDP":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _send(self, command: str) -> Optional[Dict[str, Any]]:
        """Send command and receive JSON response."""
        if not self._socket:
            raise RuntimeError("Socket not open")

        data = (command + "\n").encode("ascii")

        with self._lock:
            try:
                self._socket.sendto(data, (self.ip, self.port))
                self._last_send_time = time.time()

                try:
                    response, _ = self._socket.recvfrom(256)
                    response_str = response.decode("ascii").strip()
                    if response_str.startswith("{"):
                        return json.loads(response_str)
                except (socket.timeout, json.JSONDecodeError):
                    pass

            except Exception as e:
                print(f"[ACTUATOR-UDP] Send error: {e}")

        return None

    def _update_state(self, response: Optional[Dict[str, Any]]):
        """Update internal state from JSON response."""
        if not response:
            return
        self._state.estop = bool(response.get("e", 0))
        self._state.throttle = float(response.get("t", 0.0))
        self._state.mode = str(response.get("m", "N"))
        self._state.brake = float(response.get("b", 0.0))
        self._state.steer = float(response.get("s", 0.0))
        self._state.watchdog_active = bool(response.get("w", 0))
        self._state.timestamp = time.time()

    def _keepalive_loop(self):
        """Background thread to prevent watchdog timeout."""
        while not self._keepalive_stop.is_set():
            time.sleep(self.keepalive_interval)
            # Thread-safe read of _last_send_time
            with self._lock:
                last_send = self._last_send_time
            if time.time() - last_send >= self.keepalive_interval:
                self._send_all()

    def _send_all(self):
        """Send all current values as single command."""
        cmd = (f"A E={1 if self._estop else 0} "
               f"T={self._throttle:.3f} "
               f"M={self._mode} "
               f"B={self._brake:.3f} "
               f"S={self._steer:.3f}")
        self._update_state(self._send(cmd))

    # =========================================================================
    # Public API
    # =========================================================================

    def set_throttle(self, value: float):
        """Set throttle 0..1."""
        self._throttle = max(0.0, min(1.0, float(value)))
        self._update_state(self._send(f"T {self._throttle:.3f}"))

    def set_brake(self, value: float):
        """Set brake 0..1."""
        self._brake = max(0.0, min(1.0, float(value)))
        self._update_state(self._send(f"B {self._brake:.3f}"))

    def set_mode(self, mode: str):
        """Set drive mode: N, D, S, or R."""
        m = mode.upper()
        if m not in VALID_MODES:
            raise ValueError(f"Invalid mode '{mode}', use: {VALID_MODES}")
        self._mode = m
        self._update_state(self._send(f"M {m}"))

    def estop(self, on: bool = True):
        """Emergency stop."""
        self._estop = on
        self._update_state(self._send(f"E {1 if on else 0}"))

    def set_steer_norm(self, value: float):
        """Set steering -1..1 (left=-1, right=+1)."""
        self._steer = max(-1.0, min(1.0, float(value)))
        self._update_state(self._send(f"S {self._steer:.3f}"))

    def set_steer_deg(self, degrees: float, mech_limit_deg: float = 28.0):
        """Set steering in degrees. Converts to normalized using mechanical limit."""
        if mech_limit_deg <= 0.0:
            raise ValueError(f"mech_limit_deg must be positive, got {mech_limit_deg}")
        d = max(-mech_limit_deg, min(mech_limit_deg, float(degrees)))
        self.set_steer_norm(d / mech_limit_deg)

    def center_steering(self):
        """Center steering (send center pulse command)."""
        self._steer = 0.0
        self._update_state(self._send("C"))

    def get_state(self) -> ActuatorState:
        """Get cached actuator state."""
        return self._state

    def request_state(self) -> ActuatorState:
        """Request current state from Teensy (blocking)."""
        self._update_state(self._send("P"))
        return self._state

    def send_all(self, estop: bool = None, throttle: float = None,
                 mode: str = None, brake: float = None, steer: float = None):
        """Send all values in single UDP packet."""
        if estop is not None:
            self._estop = estop
        if throttle is not None:
            self._throttle = max(0.0, min(1.0, float(throttle)))
        if mode is not None:
            m = mode.upper()
            if m not in VALID_MODES:
                raise ValueError(f"Invalid mode '{mode}', use: {VALID_MODES}")
            self._mode = m
        if brake is not None:
            self._brake = max(0.0, min(1.0, float(brake)))
        if steer is not None:
            self._steer = max(-1.0, min(1.0, float(steer)))
        self._send_all()

    def is_connected(self) -> bool:
        """Check if socket is open."""
        return self._socket is not None

    def ping(self) -> bool:
        """Check if Teensy is responding."""
        try:
            return self._send("P") is not None
        except Exception:
            return False
