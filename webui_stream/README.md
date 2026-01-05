# AV Console

Web-based control interface for autonomous vehicle systems. Features joystick control, voice commands, and real-time telemetry.

## Features

- Touch joystick for throttle/steering
- Voice commands (cross-browser with Whisper)
- E-STOP emergency button
- Drive mode selection (N/D/S/R)
- Single controller mode (prevents conflicts)
- HTTPS for secure microphone access

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Generate SSL certificate
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
```

## Files

```
WebUI/
├── server.py        # FastAPI server
├── requirements.txt # Python dependencies
├── cert.pem         # SSL certificate
├── key.pem          # SSL private key
└── static/
    ├── index.html   # UI layout
    └── app.js       # Frontend logic
```

## Requirements

- Python 3.8+
- Teensy 4.1 with Ethernet (CAN Master)
- WiFi network connecting phone and server
