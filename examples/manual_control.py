#!/usr/bin/env python3
"""Manual vehicle control via keyboard (UP=throttle, DOWN=brake, L/R=steer override)."""

import time
import sys
import threading

sys.path.insert(0, '..')
from pynput import keyboard
from actuators import VehicleActuator

# Config
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# State
actuator = None
keys_held = set()
running = True
mode = 'N'
estop = False
brake_on = False  # Brake toggle state
steering_angle = 0.0  # Current steering (-1.0 to 1.0)
STEERING_STEP = 0.05  # Increment per key press
control_enabled = True  # Control mode toggle
listener_restart_needed = False  # Signal to restart listener
throttle_on = False  # Throttle toggle (T key)
throttle_level = 0.3  # Throttle level (0.3-0.6, set by number keys 0-4)


def on_press(key):
    global running, mode, estop, steering_angle, brake_on, control_enabled, listener_restart_needed, throttle_on, throttle_level

    keys_held.add(key)

    # Control mode toggle (works regardless of control_enabled state)
    if hasattr(key, 'char') and key.char and key.char.upper() == 'C':
        control_enabled = not control_enabled
        listener_restart_needed = True
        keys_held.clear()  # Prevent key state desync during listener restart
        print(f"\n{'='*50}")
        print(f"Control Mode: {'ON (Vehicle Control)' if control_enabled else 'OFF (Computer Use)'}")
        print(f"{'='*50}\n")
        return False  # Stop listener for restart

    # Exit (works regardless of control_enabled state)
    if key == keyboard.Key.esc:
        running = False
        return False

    # Only process vehicle controls if control is enabled
    if not control_enabled:
        return

    # Brake toggle
    if key == keyboard.Key.down:
        brake_on = not brake_on
        actuator.set_brake(1.0 if brake_on else 0.0)
        print(f"Brake: {'ON' if brake_on else 'OFF'}")

    # Steering increment/decrement
    elif key == keyboard.Key.left:
        steering_angle = max(-1.0, min(1.0, steering_angle - STEERING_STEP))
        actuator.set_steer_norm(steering_angle)
        print(f"Steering: {steering_angle:+.2f}")
    elif key == keyboard.Key.right:
        steering_angle = max(-1.0, min(1.0, steering_angle + STEERING_STEP))
        actuator.set_steer_norm(steering_angle)
        print(f"Steering: {steering_angle:+.2f}")

    # Center steering
    elif key == keyboard.Key.space:
        steering_angle = 0.0
        actuator.set_steer_norm(steering_angle)
        print(f"Steering: CENTERED (0.00)")

    # Throttle toggle (T key)
    elif hasattr(key, 'char') and key.char and key.char.upper() == 'T':
        throttle_on = not throttle_on
        status = f"ON ({throttle_level*100:.0f}%)" if throttle_on else "OFF"
        print(f"Throttle: {status}")
        if not throttle_on:
            actuator.set_throttle(0.0)

    # Throttle level control (number keys 0-4: 30%-60%)
    elif hasattr(key, 'char') and key.char in ['0', '1', '2', '3', '4']:
        throttle_level = 0.3 + (int(key.char) * 0.075)
        print(f"Throttle Level: {throttle_level*100:.0f}%")
        if throttle_on:
            print(f"  → Throttle is ON, now applying {throttle_level*100:.0f}%")

    # Cycle mode
    elif key == keyboard.Key.enter:
        modes = ['N', 'D', 'S', 'R']
        idx = modes.index(mode)
        mode = modes[(idx + 1) % len(modes)]
        actuator.set_mode(mode)
        print(f"Mode: {mode}")

    # E-STOP
    elif hasattr(key, 'char') and key.char and key.char.upper() == 'E':
        estop = not estop
        actuator.estop(estop)
        print(f"E-STOP: {'ON' if estop else 'OFF'}")


def on_release(key):
    try:
        keys_held.remove(key)
    except:
        pass


def continuous_update():
    print("\nManual control active")
    print("T=Throttle Toggle | 0-4=Throttle Level | DOWN=Brake Toggle")
    print("L/R=Steer±0.05 | SPACE=Center | ENTER=Mode | E=Estop")
    print("C=Toggle Control | ESC=Exit\n")

    while running:
        # Throttle (only active when control enabled AND throttle toggle is ON)
        if control_enabled and throttle_on:
            actuator.set_throttle(throttle_level)
        else:
            actuator.set_throttle(0.0)

        time.sleep(0.05)  # 20 Hz


def main():
    global actuator, running, listener_restart_needed

    print("\nManual Vehicle Control")
    print(f"Connecting to {SERIAL_PORT}...")

    try:
        actuator = VehicleActuator(port=SERIAL_PORT, baud=BAUD_RATE)
        print("Connected")
    except Exception as e:
        print(f"ERROR: Could not open {SERIAL_PORT}: {e}")
        sys.exit(1)

    # Initialize
    actuator.set_throttle(0.0)
    actuator.set_brake(0.0)
    actuator.set_mode("N")
    actuator.estop(False)

    # Start update thread
    threading.Thread(target=continuous_update, daemon=True).start()

    # Keyboard listener with restart capability
    while running:
        listener_restart_needed = False

        with keyboard.Listener(on_press=on_press, on_release=on_release, suppress=control_enabled) as listener:
            listener.join()

        if not listener_restart_needed:
            break

    running = False
    time.sleep(0.1)

    # Cleanup
    actuator.set_throttle(0.0)
    actuator.set_brake(1.0)
    actuator.set_steer_norm(0.0)
    actuator.close()
    print("Closed\n")


if __name__ == "__main__":
    main()
