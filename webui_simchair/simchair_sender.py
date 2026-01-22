#!/usr/bin/env python3
"""
Sim Chair UDP Sender - Sends steering wheel/pedal data to WebUI server.

This script reads from a physical sim chair (steering wheel + pedals) via pygame
and sends control data to the WebUI server via UDP on port 5006.

Protocol: CSV format "steering,throttle,brake,mode"
- steering: -1.0 to 1.0 (left to right)
- throttle: 0.0 to 1.0
- brake: 0.0 to 1.0
- mode: 0=N, 1=D, 2=S, 3=R
"""

import pygame
import time
import socket

# --- UDP setup ---
KART_IP = "100.115.20.7"   # kart's Tailscale IP (change to match your setup)
KART_PORT = 5006           # WebUI sim chair listener port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Joystick setup ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Connected to: {controller.get_name()}")
print(f"Sending to: {KART_IP}:{KART_PORT}")

driving_mode = 0

# --- Main loop ---
while True:
    pygame.event.pump()

    left_shifter = controller.get_button(0)
    right_shifter = controller.get_button(1)

    steering = controller.get_axis(0)   # Steering wheel
    throttle = controller.get_axis(6)   # Gas pedal
    brake = controller.get_axis(1)      # Brake pedal

    if left_shifter or right_shifter:
        driving_mode = max(min(driving_mode - left_shifter, driving_mode), 0)
        driving_mode = min(max(driving_mode + right_shifter, driving_mode), 3)
        time.sleep(0.25)

    throttle_norm = max(0, min(-throttle + 1, 1))   # clamp to 0..1
    brake_norm = max(0, min(-brake + 1, 1))         # clamp to 0..1

    # Print for debugging
    print(f"S: {steering:.2f} T: {throttle_norm:.2f} "
          f"B: {brake_norm:.2f} M: {driving_mode}")

    # --- Send over UDP ---
    data = f"{steering:.2f},{throttle_norm:.2f},{brake_norm:.2f},{driving_mode}"
    sock.sendto(data.encode(), (KART_IP, KART_PORT))

    time.sleep(0.05)  # 20Hz for smoother control
