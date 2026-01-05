#!/usr/bin/env python3
"""
Keyboard Control for Teensy 4.1 CAN Master (Ethernet/UDP)
Requires: pip install pynput

Controls:
  UP ARROW    - Throttle (hold)
  DOWN ARROW  - Brake (hold)
  LEFT ARROW  - Steer left (hold)
  RIGHT ARROW - Steer right (hold)
  SPACE       - Toggle E-STOP
  ENTER       - Cycle drive mode (N -> D -> S -> R -> N)
  ESC         - Exit
"""

import socket
import time
import sys
import threading
from pynput import keyboard

# ===== CONFIGURATION =====
TEENSY_IP = '192.168.13.177'
TEENSY_PORT = 5005

# ===== STATE =====
state = {
    'throttle': 0.0,
    'steer': 0.0,
    'brake': 0.0,
    'mode': 'N',
    'estop': False,
}

keys_held = set()
running = True
sock = None

# ===== UDP =====
def send(cmd):
    """Send command to Teensy"""
    try:
        sock.sendto(f"{cmd}\n".encode(), (TEENSY_IP, TEENSY_PORT))
    except:
        pass

def send_all():
    """Send all state in one command"""
    cmd = f"A E={1 if state['estop'] else 0} T={state['throttle']:.2f} M={state['mode']} B={state['brake']:.2f} S={state['steer']:.2f}"
    send(cmd)

# ===== KEYBOARD =====
def on_press(key):
    global running
    try:
        keys_held.add(key)

        if key == keyboard.Key.space:
            state['estop'] = not state['estop']
            if state['estop']:
                state['throttle'] = 0.0
                state['brake'] = 1.0
            print(f"E-STOP: {'ON' if state['estop'] else 'OFF'}")

        elif key == keyboard.Key.enter:
            modes = ['N', 'D', 'S', 'R']
            idx = modes.index(state['mode'])
            state['mode'] = modes[(idx + 1) % len(modes)]
            print(f"Mode: {state['mode']}")

        elif key == keyboard.Key.esc:
            print("\nExiting...")
            running = False
            return False
    except:
        pass

def on_release(key):
    keys_held.discard(key)

# ===== UPDATE LOOP =====
def update_loop():
    while running:
        if not state['estop']:
            state['throttle'] = 0.4 if keyboard.Key.up in keys_held else 0.0
            state['brake'] = 1.0 if keyboard.Key.down in keys_held else 0.0

            if keyboard.Key.left in keys_held:
                state['steer'] = -1.0
            elif keyboard.Key.right in keys_held:
                state['steer'] = 1.0
            else:
                state['steer'] = 0.0

        send_all()
        time.sleep(0.05)  # 20Hz

# ===== MAIN =====
def main():
    global sock, running

    print("=" * 50)
    print("  CAN Master Keyboard Control (Ethernet)")
    print("=" * 50)
    print(__doc__)
    print(f"Teensy: {TEENSY_IP}:{TEENSY_PORT}")
    print("-" * 50)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Init safe state
    send_all()
    print("Ready!\n")

    # Start update thread
    threading.Thread(target=update_loop, daemon=True).start()

    # Keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    running = False
    time.sleep(0.1)

    # Safe shutdown
    state['estop'] = True
    state['throttle'] = 0.0
    state['brake'] = 1.0
    send_all()
    sock.close()
    print("Done.")

if __name__ == "__main__":
    main()
