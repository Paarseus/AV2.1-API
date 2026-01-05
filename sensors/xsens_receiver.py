#!/usr/bin/env python3
from typing import Optional, Dict, Any
from threading import Lock, Thread
import time
import math

import xsensdeviceapi as xda


# =========================
# Callback (ring buffer)
# =========================
class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size: int = 5):
        super().__init__()
        self._max_buf = max_buffer_size
        self._buf = []
        self._lock = Lock()

    # SDK callback
    def onLiveDataAvailable(self, dev, packet):
        self._lock.acquire()
        try:
            if len(self._buf) >= self._max_buf:
                self._buf.pop(0)
            self._buf.append(xda.XsDataPacket(packet))
        finally:
            self._lock.release()

    # Helpers
    def packetAvailable(self) -> bool:
        self._lock.acquire()
        try:
            return len(self._buf) > 0
        finally:
            self._lock.release()

    def getNextPacket(self) -> xda.XsDataPacket:
        self._lock.acquire()
        try:
            if len(self._buf) == 0:
                raise RuntimeError("No packet available in buffer")
            return self._buf.pop(0)
        finally:
            self._lock.release()


# =========================
# Receiver
# =========================
class XsensReceiver:
    def __init__(self, log_file: Optional[str] = None, update_rate: int = 100):
        self.log_file = log_file
        self.update_rate = update_rate

        self.control: Optional[xda.XsControl] = None
        self.device: Optional[xda.XsDevice] = None
        self._callback: Optional[XdaCallback] = None

        self._lock = Lock()
        self._process_thread: Optional[Thread] = None
        self.is_running = False

        # Latest snapshot (thread-safe)
        self._latest_data: Dict[str, Any] = {
            "utc": None,
            "rtk_status": "NO_RTK",
            "latitude": None,
            "longitude": None,
            "altitude": None,  # ellipsoid
            "velocity": None,  # dict: east,north,up
            "acceleration": None,  # ENU free-accel if enabled
            "angular_velocity_body": None,  # dict: p,q,r
            "orientation": None,  # dict: roll,pitch,yaw_enu (from euler or quaternion)
            "quaternion": None,  # dict: w,x,y,z (raw quaternion from sensor)
            "timestamp": None,
        }

    # ---------- lifecycle ----------

    def connect(self) -> bool:
        try:
            self.control = xda.XsControl_construct()
            if not self.control:
                raise RuntimeError("Failed to construct XsControl")

            ports = xda.XsScanner_scanPorts()
            if not ports:
                raise RuntimeError("No serial ports found for Xsens devices")

            mt_port = None
            for p in ports:
                if p.deviceId().isMti() or p.deviceId().isMtig():
                    mt_port = p
                    break
            if mt_port is None:
                raise RuntimeError("No device found")

            if not self.control.openPort(mt_port.portName(), mt_port.baudrate()):
                raise RuntimeError(f"Could not open port {mt_port.portName()}")

            self.device = self.control.device(mt_port.deviceId())
            if self.device is None:
                raise RuntimeError("Could not get XsDevice handle")

            # Callback
            self._callback = XdaCallback(max_buffer_size=10)
            self.device.addCallbackHandler(self._callback)

            return True
        except Exception as e:
            print(f"Connect error: {e}")
            return False

    def configure(self) -> bool:
        """
        Force ENU for Orientation (Euler) and Velocity. Enable free-acc ENU and body gyro.
        """
        try:
            print("Putting device into configuration mode...")
            if not self.device.gotoConfig():
                raise RuntimeError("Could not put device into configuration mode.")

            if not self.device.deviceId().isGnss():
                raise RuntimeError("Device is not a GNSS device.")

            # Set filter profile to General_RTK (ID 0, no magnetometer, suitable for RTK)
            # Note: Not using GeneralMag_RTK (ID 2) due to magnetic interference from metal vehicle
            # General_RTK relies on GNSS+IMU only, heading converges automatically when moving
            print("Setting filter profile to General_RTK (ID 0, no mag)...")
            try:
                if self.device.setOnboardFilterProfile(0):
                    print("Successfully set filter profile to General_RTK (ID 0)")
                    # Verify the profile was actually set
                    try:
                        current_profile = self.device.onboardFilterProfile()
                        print(f"Verified: Current filter profile is {current_profile}")
                    except Exception:
                        print("Note: Could not verify filter profile (not critical)")
                else:
                    print("Warning: Could not set filter profile, using device default")
            except Exception as e:
                print(f"Warning: Filter profile setting not supported on this device: {e}")

            print("Configuring the device (ENU quaternion + velocity)...")
            cfg = xda.XsOutputConfigurationArray()

            # Basic timing/status
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_StatusWord, 0))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_UtcTime, 0))

            # --- ENU orientation + velocity (critical for heading & pursuit) ---
            # Using Quaternion (SDK recommended for GNSS/INS, no gimbal lock, more stable)
            # SDK can still provide orientationEuler() from quaternion data
            ORI_QUAT_ENU = xda.XDI_Quaternion | xda.XDI_CoordSysEnu
            VEL_ENU = xda.XDI_VelocityXYZ | xda.XDI_CoordSysEnu
            cfg.push_back(xda.XsOutputConfiguration(ORI_QUAT_ENU, self.update_rate))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, self.update_rate))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, self.update_rate))
            cfg.push_back(xda.XsOutputConfiguration(VEL_ENU, self.update_rate))

            # Free acceleration in ENU (recommended for dynamics)
            FREEACC_ENU = xda.XDI_FreeAcceleration | xda.XDI_CoordSysEnu
            cfg.push_back(xda.XsOutputConfiguration(FREEACC_ENU, self.update_rate))

            # Gyros: calibrated body rates (p,q,r). (No ENU flag here.)
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, self.update_rate))

            if not self.device.setOutputConfiguration(cfg):
                raise RuntimeError("Could not configure the device.")

            # On-device log file (optional)
            if self.log_file:
                print(f"Creating log file: {self.log_file}")
                if self.device.createLogFile(self.log_file) != xda.XRV_OK:
                    print(f"Warning: Failed to create log file {self.log_file}")

            return True

        except Exception as e:
            print(f"Configuration error: {e}")
            return False

    def start(
        self,
        wait_for_rtk_fixed: bool = False,
        rtk_timeout: float = 60.0,
        fail_on_timeout: bool = False,
    ) -> bool:
        """
        Start streaming. Optionally block until RTK FIXED before returning.

        Args:
            wait_for_rtk_fixed: If True, block until RTK FIXED (bit 28) is seen.
            rtk_timeout: Seconds to wait for FIXED before giving up.
            fail_on_timeout: If True and FIXED not achieved by rtk_timeout,
                             stop and return False; else continue anyway.
        """
        try:
            if not self.connect():
                return False
            if not self.configure():
                return False

            print("Putting device into measurement mode...")
            if not self.device.gotoMeasurement():
                raise RuntimeError("Could not put device into measurement mode.")

            if self.log_file:
                print("Starting on-device recording...")
                if not self.device.startRecording():
                    print("Warning: Failed to start recording.")

            self.is_running = True
            self._process_thread = Thread(target=self._process_data, daemon=True)
            self._process_thread.start()

            if wait_for_rtk_fixed:
                print(f"Waiting for RTK FIXED (timeout: {rtk_timeout:.0f}s)...")
                got_fix = self.wait_for_rtk_fix(timeout=rtk_timeout)
                if not got_fix:
                    msg = "RTK FIXED not achieved within timeout."
                    if fail_on_timeout:
                        print("[XSENS] ERROR: " + msg + " Stopping.")
                        self.stop()
                        return False
                    else:
                        print("[XSENS] WARN: " + msg + " Continuing without FIXED.")
            else:
                print("GPS receiver started.")

            return True

        except Exception as e:
            print(f"Start error: {e}")
            return False

    def stop(self):
        try:
            self.is_running = False
            if self._process_thread is not None:
                self._process_thread.join(timeout=2.0)
                self._process_thread = None

            if self.device:
                # Stop on-device logging if active
                try:
                    self.device.stopRecording()
                    self.device.closeLogFile()
                except Exception:
                    pass

                if self._callback:
                    self.device.removeCallbackHandler(self._callback)

            if self.control:
                self.control.close()
                self.control = None
                self.device = None
                self._callback = None

            print("XSens receiver stopped.")
        except Exception as e:
            print(f"Stop error: {e}")

    # ---------- background decode ----------

    def _process_data(self):
        """Continuously pop packets from callback, decode into _latest_data."""
        while self.is_running and self._callback:
            if not self._callback.packetAvailable():
                time.sleep(0.001)
                continue

            pkt = self._callback.getNextPacket()
            now = time.time()

            try:
                # Status word -> RTK status decode (bit 28 FIXED, bit 27 FLOAT)
                if pkt.containsStatus():
                    status = pkt.status()
                    if status & (1 << 28):
                        rtk = "RTK_FIXED"
                    elif status & (1 << 27):
                        rtk = "RTK_FLOAT"
                    else:
                        rtk = "NO_RTK"
                else:
                    rtk = self._latest_data["rtk_status"]

                # UTC
                utc_dict = None
                if pkt.containsUtcTime():
                    utc = pkt.utcTime()
                    utc_dict = {
                        "year": utc.m_year,
                        "month": utc.m_month,
                        "day": utc.m_day,
                        "hour": utc.m_hour,
                        "minute": utc.m_minute,
                        "second": utc.m_second,
                        "nano": utc.m_nano,
                        "valid": utc.m_valid,
                    }

                # Position
                lat = self._latest_data["latitude"]
                lon = self._latest_data["longitude"]
                alt = self._latest_data["altitude"]
                if pkt.containsLatitudeLongitude():
                    lat, lon = pkt.latitudeLongitude()

                if pkt.containsAltitude():
                    alt = pkt.altitude()

                # Velocity (ENU)
                vel_dict = self._latest_data["velocity"]
                if pkt.containsVelocity():
                    vx, vy, vz = pkt.velocity(xda.XDI_CoordSysEnu)
                    vel_dict = {"east": vx, "north": vy, "up": vz}

                # Free acceleration (ENU)
                acc_dict = self._latest_data["acceleration"]
                if pkt.containsFreeAcceleration():
                    ax, ay, az = pkt.freeAcceleration()
                    acc_dict = {"east": ax, "north": ay, "up": az}

                # Gyros (body)
                avb_dict = self._latest_data["angular_velocity_body"]
                if pkt.containsCalibratedGyroscopeData():
                    gx, gy, gz = pkt.calibratedGyroscopeData()
                    avb_dict = {"p": gx, "q": gy, "r": gz}

                # Orientation (quaternion primary, euler derived)
                # Device configured for Quaternion output (more stable, SDK recommended)
                # SDK can provide both quaternion and euler from same packet
                ori_dict = self._latest_data["orientation"]
                quat_dict = self._latest_data["quaternion"]
                if pkt.containsOrientation():
                    # Extract quaternion (primary, raw from sensor)
                    q = pkt.orientationQuaternion()  # ENU because configured
                    quat_dict = {"w": q[0], "x": q[1], "y": q[2], "z": q[3]}

                    # Extract euler (SDK converts from quaternion)
                    e = pkt.orientationEuler()  # ENU because configured
                    ori_dict = {"roll": e.x(), "pitch": e.y(), "yaw_enu": e.z()}

                # Commit snapshot
                with self._lock:
                    self._latest_data.update(
                        {
                            "utc": utc_dict,
                            "rtk_status": rtk,
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": alt,
                            "velocity": vel_dict,
                            "acceleration": acc_dict,
                            "angular_velocity_body": avb_dict,
                            "orientation": ori_dict,
                            "quaternion": quat_dict,
                            "timestamp": now,
                        }
                    )
            except Exception as ex:
                # Keep the loop alive; print once per hiccup
                print(f"Packet decode error: {ex}")

    # ---------- getters ----------

    def get_all_data(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest_data)

    def get_current_position(self) -> Optional[tuple]:
        with self._lock:
            lat = self._latest_data["latitude"]
            lon = self._latest_data["longitude"]
            alt = self._latest_data["altitude"]
            if lat is None or lon is None:
                return None
            return (lat, lon, alt)

    def get_velocity(self) -> Optional[Dict[str, float]]:
        with self._lock:
            return self._latest_data["velocity"]

    def get_speed(self) -> float:
        """
        Returns scalar ground speed in m/s (horizontal velocity magnitude).
        Computed from ENU velocity: sqrt(east^2 + north^2)
        """
        with self._lock:
            vel = self._latest_data["velocity"]
            if vel is None:
                return 0.0
            east = vel.get('east', 0.0)
            north = vel.get('north', 0.0)
            return math.sqrt(east**2 + north**2)

    def get_rtk_status(self) -> str:
        with self._lock:
            return self._latest_data["rtk_status"]

    def get_utc_time(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest_data["utc"]

    def get_heading_enu(self) -> Optional[float]:
        """
        Returns ENU yaw in radians for pure pursuit controller.
        0 rad = East, π/2 rad = North, π rad = West, -π/2 rad = South
        Rotation: Counter-clockwise positive
        """
        with self._lock:
            ori = self._latest_data["orientation"]
            if not ori:
                return None
            yaw_enu = ori.get("yaw_enu")
            if yaw_enu is None:
                return None
            # Convert from degrees to radians
            return math.radians(yaw_enu)

    def get_heading_nav(self) -> Optional[float]:
        """
        Returns navigation/compass heading in degrees.
        0° = North, 90° = East, 180° = South, 270° = West
        Rotation: Clockwise positive
        Computed from ENU yaw via: heading = (90 - yaw_enu) mod 360
        """
        with self._lock:
            ori = self._latest_data["orientation"]
            if not ori:
                return None
            yaw_enu = ori.get("yaw_enu")
            if yaw_enu is None:
                return None
            return (90.0 - yaw_enu) % 360.0

    def get_yaw_rate(self) -> Optional[float]:
        """
        Body yaw rate r [rad/s] (about body Z). If the unit is mounted level,
        r ≈ yaw rate about ENU Up. For full correctness, rotate to ENU.
        """
        with self._lock:
            avb = self._latest_data["angular_velocity_body"]
            return avb["r"] if avb else None

    def get_quaternion(self) -> Optional[Dict[str, float]]:
        """
        Returns orientation quaternion in ENU frame.
        Format: {"w": w, "x": x, "y": y, "z": z}
        Quaternion represents rotation from ENU world frame to body frame.
        More numerically stable than Euler angles (no gimbal lock).
        """
        with self._lock:
            return self._latest_data["quaternion"]

    # ---------- utilities ----------

    def wait_for_rtk_fix(self, timeout: float = 60.0) -> bool:
        """Block until RTK FIXED is reported or timeout expires."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.get_rtk_status() == "RTK_FIXED":
                return True
            time.sleep(0.2)
        return False

    def is_fresh(self, max_age_s: float = 0.5) -> bool:
        with self._lock:
            ts = self._latest_data["timestamp"]
        return (ts is not None) and ((time.time() - ts) <= max_age_s)


if __name__ == "__main__":
    print("="*70)
    print("XSENS MTi-680G TEST - Quaternion Configuration")
    print("Filter Profile: General_RTK (ID 0, no magnetometer)")
    print("Orientation: Quaternion (SDK recommended for GNSS/INS)")
    print("Coordinate System: ENU (East-North-Up)")
    print("="*70)

    # Test duration in seconds (configurable)
    TEST_DURATION = 60.0  # 60 seconds = 1 minute

    xs = XsensReceiver(log_file=None, update_rate=100)
    if xs.start(wait_for_rtk_fixed=False):
        print(f"\nStreaming {int(TEST_DURATION)} seconds of samples...")
        print("Press Ctrl+C to stop early\n")
        t0 = time.time()
        iteration = 0
        try:
            while time.time() - t0 < TEST_DURATION:
                pos = xs.get_current_position()
                vel = xs.get_velocity()
                hdg_nav = xs.get_heading_nav()
                hdg_enu = xs.get_heading_enu()
                quat = xs.get_quaternion()
                rtk = xs.get_rtk_status()

                elapsed = time.time() - t0

                print(
                    f"[{elapsed:5.1f}s] RTK={rtk}"
                    f"{', Pos:'+str(pos) if pos else ''}"
                    f"{', Vel:'+str(vel) if vel else ''}"
                    f"{', Nav: %.1f°' % hdg_nav if hdg_nav is not None else ''}"
                    f"{', ENU: %.3f rad' % hdg_enu if hdg_enu is not None else ''}"
                    f"{', Quat: (%.3f, %.3f, %.3f, %.3f)' % (quat['w'], quat['x'], quat['y'], quat['z']) if quat else ''}"
                )
                time.sleep(0.2)
                iteration += 1
        except KeyboardInterrupt:
            print(f"\n\nTest interrupted by user after {time.time() - t0:.1f} seconds")

        xs.stop()
        print("\n" + "="*70)
        print(f"Test completed: {iteration} samples in {time.time() - t0:.1f} seconds")
        print("="*70)
