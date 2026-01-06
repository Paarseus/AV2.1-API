#!/usr/bin/env python3
"""
Tests for VehicleActuatorUDP bug fixes.

Tests cover:
1. _last_send_time initialization
2. Mode validation consistency in send_all()
3. Thread-safe access to _last_send_time
4. Division by zero prevention in set_steer_deg()
"""

import sys
import os
import time
import unittest
from unittest.mock import Mock, patch, MagicMock
import socket

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from actuators.udp import VehicleActuatorUDP, ActuatorState, VALID_MODES


class TestUDPBugFixes(unittest.TestCase):
    """Test cases for UDP actuator bug fixes."""

    def setUp(self):
        """Set up test fixtures."""
        # Mock socket to prevent actual network calls
        self.socket_patcher = patch('socket.socket')
        self.mock_socket_class = self.socket_patcher.start()
        self.mock_socket = MagicMock()
        self.mock_socket_class.return_value = self.mock_socket
        
        # Mock socket methods
        self.mock_socket.recvfrom.side_effect = socket.timeout()
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.socket_patcher.stop()

    def test_last_send_time_initialization(self):
        """Test that _last_send_time is properly initialized to prevent immediate keepalive."""
        # Create actuator with keepalive enabled
        start_time = time.time()
        actuator = VehicleActuatorUDP(keepalive=True, keepalive_interval=0.2)
        
        # _last_send_time should be initialized to current time, not 0.0
        self.assertGreater(actuator._last_send_time, 0.0,
                          "_last_send_time should not be 0.0")
        self.assertAlmostEqual(actuator._last_send_time, start_time, delta=0.1,
                              msg="_last_send_time should be close to initialization time")
        
        actuator.close()

    def test_send_all_mode_validation(self):
        """Test that send_all() raises ValueError for invalid modes."""
        actuator = VehicleActuatorUDP(keepalive=False)
        
        # Valid modes should work
        for mode in VALID_MODES:
            try:
                actuator.send_all(mode=mode)
            except ValueError:
                self.fail(f"send_all() should accept valid mode '{mode}'")
        
        # Invalid modes should raise ValueError
        invalid_modes = ["X", "Invalid", "123", ""]
        for invalid_mode in invalid_modes:
            with self.assertRaises(ValueError,
                                 msg=f"send_all() should raise ValueError for invalid mode '{invalid_mode}'"):
                actuator.send_all(mode=invalid_mode)
        
        actuator.close()

    def test_set_steer_deg_zero_limit(self):
        """Test that set_steer_deg() handles zero mechanical limit gracefully."""
        actuator = VehicleActuatorUDP(keepalive=False)
        
        # Should raise ValueError when mech_limit_deg is 0
        with self.assertRaises(ValueError,
                             msg="set_steer_deg() should raise ValueError when mech_limit_deg is 0"):
            actuator.set_steer_deg(10.0, mech_limit_deg=0.0)
        
        # Should raise ValueError when mech_limit_deg is negative
        with self.assertRaises(ValueError,
                             msg="set_steer_deg() should raise ValueError when mech_limit_deg is negative"):
            actuator.set_steer_deg(10.0, mech_limit_deg=-5.0)
        
        actuator.close()

    def test_set_steer_deg_normal_operation(self):
        """Test that set_steer_deg() works correctly with valid inputs."""
        actuator = VehicleActuatorUDP(keepalive=False)
        
        # Test with default mechanical limit
        actuator.set_steer_deg(14.0)  # Half of 28 degrees
        self.assertAlmostEqual(actuator._steer, 0.5, places=2,
                              msg="Steering should be normalized correctly")
        
        # Test with custom mechanical limit
        actuator.set_steer_deg(10.0, mech_limit_deg=20.0)
        self.assertAlmostEqual(actuator._steer, 0.5, places=2,
                              msg="Steering should be normalized with custom limit")
        
        # Test clamping at limits
        actuator.set_steer_deg(50.0, mech_limit_deg=28.0)
        self.assertAlmostEqual(actuator._steer, 1.0, places=2,
                              msg="Steering should clamp to +1.0 at max")
        
        actuator.set_steer_deg(-50.0, mech_limit_deg=28.0)
        self.assertAlmostEqual(actuator._steer, -1.0, places=2,
                              msg="Steering should clamp to -1.0 at min")
        
        actuator.close()

    def test_mode_validation_consistency(self):
        """Test that mode validation is consistent across set_mode() and send_all()."""
        actuator = VehicleActuatorUDP(keepalive=False)
        
        # Both methods should raise ValueError for invalid mode
        with self.assertRaises(ValueError):
            actuator.set_mode("INVALID")
        
        with self.assertRaises(ValueError):
            actuator.send_all(mode="INVALID")
        
        actuator.close()

    def test_keepalive_does_not_fire_immediately(self):
        """Test that keepalive thread doesn't send immediately on start."""
        # Create actuator with short keepalive interval
        actuator = VehicleActuatorUDP(keepalive=True, keepalive_interval=0.3)
        
        # Record initial send count
        initial_calls = self.mock_socket.sendto.call_count
        
        # Wait less than keepalive_interval
        time.sleep(0.1)
        
        # Should not have sent keepalive yet
        calls_after_short_wait = self.mock_socket.sendto.call_count
        self.assertEqual(calls_after_short_wait, initial_calls,
                        "Keepalive should not fire before interval expires")
        
        actuator.close()

    def test_thread_safe_last_send_time_access(self):
        """Test that _last_send_time is accessed in a thread-safe manner."""
        actuator = VehicleActuatorUDP(keepalive=True, keepalive_interval=0.2)
        
        # Simulate concurrent access by multiple operations
        for _ in range(10):
            actuator.set_throttle(0.5)
            time.sleep(0.01)
            # Read _last_send_time (should not crash or cause issues)
            last_time = actuator._last_send_time
            self.assertIsInstance(last_time, float)
            self.assertGreater(last_time, 0.0)
        
        actuator.close()


class TestActuatorState(unittest.TestCase):
    """Test ActuatorState dataclass."""

    def test_default_values(self):
        """Test that ActuatorState has correct default values."""
        state = ActuatorState()
        self.assertFalse(state.estop)
        self.assertEqual(state.throttle, 0.0)
        self.assertEqual(state.mode, "N")
        self.assertEqual(state.brake, 0.0)
        self.assertEqual(state.steer, 0.0)
        self.assertFalse(state.watchdog_active)
        self.assertEqual(state.timestamp, 0.0)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
