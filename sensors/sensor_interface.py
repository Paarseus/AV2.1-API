"""
Sensor Interface Base Class
Common infrastructure for all sensors (Camera, LIDAR, GPS, IMU, etc.)

Provides:
    - Standard lifecycle (connect, start, stop, disconnect)
    - Thread management for continuous data capture
    - Callback registration for event-driven processing
    - Thread-safe state management

Usage:
    class MySensor(SensorInterface):
        def connect(self) -> bool:
            # Hardware-specific initialization

        def _capture_loop(self):
            # Hardware-specific data capture
            while self._running:
                data = capture_from_hardware()
                timestamp = time.time()
                self._notify_callbacks(data, timestamp)
"""

from abc import ABC, abstractmethod
import threading
import time
import logging
from typing import Callable, List, Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SensorInterface(ABC):
    """
    Abstract base class for all sensor types

    Implements common lifecycle, threading, and callback infrastructure
    Subclasses implement hardware-specific connect() and _capture_loop()
    """

    def __init__(self, sensor_id: str = "sensor"):
        """
        Initialize sensor interface

        Args:
            sensor_id: Unique identifier for this sensor instance
        """
        self.sensor_id = sensor_id

        # Connection and streaming state
        self._connected = False
        self._running = False

        # Thread management
        self._capture_thread = None

        # Callback management
        self._callbacks: List[Callable[[Any, float], None]] = []
        self._callback_lock = threading.Lock()

    @abstractmethod
    def connect(self) -> bool:
        """
        Initialize hardware connection

        Returns:
            True if successful, False otherwise

        Implementation must:
            - Initialize hardware communication
            - Set self._connected = True on success
            - Return True on success, False on failure
        """
        pass

    @abstractmethod
    def _capture_loop(self):
        """
        Background thread for continuous data capture

        Implementation must:
            - Run while self._running is True
            - Capture data from hardware
            - Call self._notify_callbacks(data, timestamp) for new data
            - Handle exceptions gracefully
            - Exit cleanly when self._running becomes False
        """
        pass

    def start(self) -> bool:
        """
        Start data capture thread

        Returns:
            True if successful, False otherwise
        """
        if not self._connected:
            logger.error(f"{self.sensor_id}: Cannot start - not connected")
            return False

        if self._running:
            logger.warning(f"{self.sensor_id}: Already running")
            return True

        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"{self.sensor_id}_capture"
        )
        self._capture_thread.start()
        logger.info(f"{self.sensor_id}: Capture thread started")
        return True

    def stop(self):
        """Stop data capture thread"""
        if self._running:
            logger.info(f"{self.sensor_id}: Stopping capture...")
            self._running = False
            if self._capture_thread:
                self._capture_thread.join(timeout=2.0)
                if self._capture_thread.is_alive():
                    logger.warning(f"{self.sensor_id}: Thread did not stop cleanly")
            logger.info(f"{self.sensor_id}: Stopped")

    def disconnect(self):
        """
        Release hardware resources

        Default implementation calls stop() and resets connection state
        Subclasses can override for additional cleanup
        """
        self.stop()
        if self._connected:
            self._connected = False
            logger.info(f"{self.sensor_id}: Disconnected")

    def is_connected(self) -> bool:
        """Check if sensor is connected to hardware"""
        return self._connected

    def is_running(self) -> bool:
        """Check if data capture thread is running"""
        return self._running

    def register_callback(self, callback: Callable[[Any, float], None]):
        """
        Register callback for new data events

        Args:
            callback: Function with signature callback(data, timestamp)
                     Called whenever new data is captured

        Example:
            def on_new_data(data, timestamp):
                print(f"Got data at {timestamp}")

            sensor.register_callback(on_new_data)
            sensor.start()  # Callback invoked automatically
        """
        with self._callback_lock:
            self._callbacks.append(callback)
            logger.info(f"{self.sensor_id}: Registered callback {callback.__name__}")

    def unregister_callback(self, callback: Callable[[Any, float], None]):
        """Remove a previously registered callback"""
        with self._callback_lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
                logger.info(f"{self.sensor_id}: Unregistered callback {callback.__name__}")

    def clear_callbacks(self):
        """Remove all registered callbacks"""
        with self._callback_lock:
            count = len(self._callbacks)
            self._callbacks.clear()
            logger.info(f"{self.sensor_id}: Cleared {count} callbacks")

    def _notify_callbacks(self, data: Any, timestamp: float):
        """
        Invoke all registered callbacks with new data

        Called by subclass _capture_loop() when new data arrives
        Callbacks are invoked in registration order

        Args:
            data: Sensor-specific data (np.ndarray, dict, etc.)
            timestamp: Timestamp of data capture
        """
        with self._callback_lock:
            callbacks = self._callbacks.copy()

        for callback in callbacks:
            try:
                callback(data, timestamp)
            except Exception as e:
                logger.error(f"{self.sensor_id}: Callback {callback.__name__} failed - {e}")


# Example usage and testing
if __name__ == "__main__":
    """
    Example sensor implementation demonstrating SensorInterface usage
    """
    import random

    class DummySensor(SensorInterface):
        """Example sensor that generates random data"""

        def __init__(self, sensor_id="dummy"):
            super().__init__(sensor_id)
            self._data_lock = threading.Lock()
            self._latest_data = None
            self._latest_timestamp = 0.0

        def connect(self) -> bool:
            """Simulated connection"""
            logger.info(f"{self.sensor_id}: Connecting to dummy hardware...")
            time.sleep(0.5)
            self._connected = True
            logger.info(f"{self.sensor_id}: Connected")
            return True

        def _capture_loop(self):
            """Generate random data at 10Hz"""
            logger.info(f"{self.sensor_id}: Capture loop started")
            while self._running:
                # Simulate data capture
                data = random.random()
                timestamp = time.time()

                # Store latest
                with self._data_lock:
                    self._latest_data = data
                    self._latest_timestamp = timestamp

                # Notify callbacks
                self._notify_callbacks(data, timestamp)

                time.sleep(0.1)  # 10Hz

            logger.info(f"{self.sensor_id}: Capture loop exited")

        def get_latest(self):
            """Get latest data (polling interface)"""
            with self._data_lock:
                return self._latest_data, self._latest_timestamp

    # Test the interface
    print("="*60)
    print("SENSOR INTERFACE TEST")
    print("="*60)

    # Create sensor
    sensor = DummySensor("test_sensor")

    # Register callbacks
    def callback1(data, timestamp):
        print(f"Callback 1: data={data:.3f}, time={timestamp:.2f}")

    def callback2(data, timestamp):
        print(f"Callback 2: Got value {data:.3f}")

    sensor.register_callback(callback1)
    sensor.register_callback(callback2)

    # Connect and start
    print("\nConnecting...")
    if sensor.connect():
        print(f"Connected: {sensor.is_connected()}")

        print("\nStarting...")
        if sensor.start():
            print(f"Running: {sensor.is_running()}")

            # Let it run for 3 seconds
            print("\nReceiving data for 3 seconds...\n")
            time.sleep(3)

            # Also test polling
            print("\nPolling latest data:")
            data, timestamp = sensor.get_latest()
            print(f"Latest: data={data:.3f}, time={timestamp:.2f}")

            # Stop
            print("\nStopping...")
            sensor.stop()
            print(f"Running: {sensor.is_running()}")

    # Disconnect
    print("\nDisconnecting...")
    sensor.disconnect()
    print(f"Connected: {sensor.is_connected()}")

    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
