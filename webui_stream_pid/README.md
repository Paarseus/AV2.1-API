# AV Console (PID Cruise Control Version)

Web-based control interface for autonomous vehicle systems. Features joystick control, voice commands, real-time telemetry, and **PID cruise control** for maintaining consistent speeds.

## Features

- Touch joystick for throttle/steering
- **Two control modes:**
  - **Direct Mode:** Joystick controls throttle directly (original behavior)
  - **Cruise Control Mode:** PID maintains target speed automatically
- Voice commands (cross-browser with Whisper)
- E-STOP emergency button
- Drive mode selection (N/D/S/R)
- Live GPS speed display
- Single controller mode (prevents conflicts)
- HTTPS for secure microphone access

## Cruise Control

### How It Works

In cruise control mode, the system uses GPS speed feedback from the Xsens IMU to automatically adjust throttle via a PID controller, maintaining a consistent speed regardless of terrain.

```
Direct Mode:     Joystick Y -> Throttle -> Teensy
Cruise Mode:     Target Speed -> PID -> Throttle -> Teensy
                       ^
                 GPS Speed (Xsens)
```

### UI Controls

```
┌──────────────────────────────────────┐
│  [DIRECT]  [CRUISE]                  │  <- Mode toggle
├──────────────────────────────────────┤
│  Cruise Control:                     │
│  ┌────────────────────────────────┐  │
│  │  Target: [1.0 m/s]  [-] [+]    │  │
│  │  [ENGAGE]  [CANCEL]            │  │
│  │  Status: Engaged / Disengaged  │  │
│  └────────────────────────────────┘  │
│                                      │
│  Speed: 0.95 m/s | Throttle: 32%     │
└──────────────────────────────────────┘
```

### Usage

1. Click **CRUISE** to switch to cruise control mode
2. Use **[+]** / **[-]** to set target speed (0.0 - 2.0 m/s)
3. Click **ENGAGE** to start cruise control
4. Use joystick X-axis for steering (Y-axis ignored)
5. Click **CANCEL** or switch to **DIRECT** to disengage

### Safety Features

- E-STOP automatically cancels cruise
- Cruise auto-cancels on WebSocket disconnect
- GPS timeout (0.5s stale data) cancels cruise
- Speed limited to 2.0 m/s maximum
- Cruise disabled when GPS unavailable

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Generate SSL certificate (if not present)
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=AVConsole'

# Run server
python3 server.py
```

Open `https://<your-ip>:8000` on your phone and accept the certificate warning.

## Voice Commands

| Command | Action |
|---------|--------|
| "stop" / "halt" | E-STOP |
| "go" / "forward" | Throttle for 2s |
| "left" | Steer left for 1s |
| "right" | Steer right for 1s |
| "neutral" | Mode N |
| "drive" | Mode D |
| "reverse" | Mode R |
| "sport" | Mode S |

## Configuration

Edit `server.py` to change:

```python
TEENSY_IP = '192.168.13.177'  # Teensy address
TEENSY_PORT = 5005            # UDP port
WEB_PORT = 8000               # Web server port
MAX_THROTTLE = 0.4            # Safety limit (0.0-1.0)
MAX_CRUISE_SPEED = 2.0        # Max cruise speed (m/s)
CONTROL_RATE_HZ = 20          # PID control loop rate
```

PID gains (from config/default.yaml):
```python
kp = 0.34
ki = 0.15
kd = 0.065
```

## Files

```
webui_stream_pid/
├── server.py        # FastAPI server with PID cruise control
├── requirements.txt # Python dependencies
├── cert.pem         # SSL certificate
├── key.pem          # SSL private key
└── static/
    ├── index.html   # UI layout (with cruise control panel)
    └── app.js       # Frontend logic (with cruise handlers)
```

## Requirements

- Python 3.8+
- Teensy 4.1 with Ethernet (CAN Master)
- Xsens MTi-680G GPS/IMU (for cruise control)
- WiFi network connecting phone and server
