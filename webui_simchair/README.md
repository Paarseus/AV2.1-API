# WebUI with Sim Chair Support

Unified control interface for phone (WebSocket) and sim chair (UDP) with automatic priority switching.

## When to Use This vs Direct Sim Chair

| Use Case | Solution |
|----------|----------|
| Sim chair + phone video/E-STOP/telemetry | **Use this** (`webui_simchair/`) |
| Sim chair only, no phone needed | Use `firmware/Simchair/simchair2udp.py` directly to Teensy |
| Phone control only | Use `webui_stream/` |

## Architecture

```
Phone ──WebSocket:8000──┐
                        ├──► server.py ──UDP:5005──► Teensy
Sim Chair ────UDP:5006──┘        │
                                 └──► WebRTC Video ──► Phone
```

**Note:** Phone must be connected for sim chair to work. The phone WebSocket drives the control loop at 20Hz and provides video/E-STOP capability.

## Features

- **Priority-based control**: Sim chair takes priority when active
- **Automatic fallback**: Falls back to phone after 500ms of no sim chair input
- **E-STOP always works**: Phone E-STOP works regardless of control source
- **WebRTC video**: Live camera feed to phone
- **Voice control**: Whisper transcription for voice commands

## Usage

### On Kart (run server):
```bash
cd webui_simchair && python server.py
```

Expected output:
```
Loading Whisper model...
Whisper ready
Connecting to RealSense camera...
Camera ready: 1280x720 @ 30fps
[SIMCHAIR] Listening on UDP port 5006
AV Console: https://0.0.0.0:8000
Teensy: 192.168.13.177:5005
```

### On Sim Chair PC (run sender):
```bash
python simchair_sender.py
```

Edit `KART_IP` in `simchair_sender.py` to match your kart's IP address.

### On Phone (view + E-STOP):
Open `https://<kart-tailscale-ip>:8000`

## Priority System

| Scenario | Behavior |
|----------|----------|
| Sim chair active | Sim chair controls vehicle |
| Sim chair inactive (>500ms) | Phone controls vehicle |
| E-STOP pressed (any source) | Vehicle stops immediately |
| Teensy released | External control (autonomous mode) |

## Protocol

**Format:** `steering,throttle,brake,mode`

**Example:** `0.50,0.30,0.00,1`

| Field | Type | Range |
|-------|------|-------|
| steering | float | -1.0 to 1.0 (left to right) |
| throttle | float | 0.0 to 1.0 |
| brake | float | 0.0 to 1.0 |
| mode | int | 0=N, 1=D, 2=S, 3=R |

## Telemetry

The phone WebUI displays:
- Current control values (T, S, B)
- Control source (`phone` or `simchair`)
- E-STOP status
- Drive mode

## Testing

### Test with netcat (no sim chair hardware):
```bash
# Send a single command
echo "0.5,0.3,0,1" | nc -u <kart-ip> 5006

# Continuous test loop
while true; do echo "0.0,0.2,0.0,1" | nc -u <kart-ip> 5006; sleep 0.1; done
```

### Verify automatic fallback:
1. Start sim chair sender
2. Confirm phone shows `source: simchair`
3. Stop sim chair sender (Ctrl+C)
4. Wait 0.5 seconds
5. Confirm phone shows `source: phone`

## Files

| File | Purpose |
|------|---------|
| `server.py` | WebUI server with sim chair UDP listener |
| `simchair_sender.py` | Reads sim chair hardware, sends UDP |
| `static/` | Web frontend (HTML, JS, manifest) |
| `cert.pem`, `key.pem` | SSL certificates for HTTPS |
| `requirements.txt` | Python dependencies |

## Configuration

Edit these constants in `server.py`:
- `TEENSY_IP`: Teensy's IP address (default: 192.168.13.177)
- `TEENSY_PORT`: Teensy's UDP port (default: 5005)
- `WEB_PORT`: WebUI HTTPS port (default: 8000)
- `MAX_THROTTLE`: Maximum throttle from phone (default: 0.4)
- `SIMCHAIR_PORT`: Sim chair UDP port (default: 5006)
- `SIMCHAIR_TIMEOUT`: Fallback timeout in seconds (default: 0.5)

Edit these constants in `simchair_sender.py`:
- `KART_IP`: Kart's IP address (Tailscale or local)
- `KART_PORT`: Must match SIMCHAIR_PORT in server.py (default: 5006)

## Generating SSL Certificates

If you need new certificates:
```bash
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=AVConsole'
```

## Dependencies

Same as `webui_stream/`:
```
fastapi
uvicorn[standard]
websockets
faster-whisper
python-multipart
aiortc>=1.6.0
av>=10.0.0
opencv-python>=4.8.0
pyrealsense2>=2.50.0
numpy
```

Sim chair sender additionally requires:
```
pygame
```
