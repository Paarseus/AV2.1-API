#!/usr/bin/env python3
"""
AV Console - Web UI Server with WebRTC Camera Streaming

WebSocket bridge to Teensy via VehicleActuatorUDP + Whisper voice transcription.
WebRTC video streaming from Intel RealSense camera.
"""

import sys
import tempfile
import os
import time
import asyncio
import fractions
from pathlib import Path

import cv2
import numpy as np

# Add FINALE to path for imports
FINALE_PATH = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FINALE_PATH))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse, Response
from faster_whisper import WhisperModel
from actuators import VehicleActuatorUDP

# WebRTC imports
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame

# Camera import
from sensors.camera_interface import RGBDCamera

# Configuration
TEENSY_IP = "192.168.13.177"
TEENSY_PORT = 5005
WEB_PORT = 8000
MAX_THROTTLE = 0.4

# Whisper model
print("Loading Whisper model...")
whisper_model = WhisperModel("tiny", device="cpu", compute_type="int8")
print("Whisper ready")

# Actuator (keepalive disabled - WebUI sends at 50ms)
actuator = VehicleActuatorUDP(ip=TEENSY_IP, port=TEENSY_PORT, keepalive=False)

# Camera (shared, thread-safe)
print("Connecting to RealSense camera...")
camera = RGBDCamera(width=1280, height=720, fps=30)
if camera.connect():
    camera.start()
    print("Camera ready: 1280x720 @ 30fps")
else:
    print("WARNING: Camera unavailable - streaming disabled")
    camera = None

# WebRTC peer connections (per-client)
peer_connections = {}


async def setup_webrtc(websocket, sdp):
    """Setup WebRTC peer connection (runs in background)."""
    try:
        rtc = RTCPeerConnection()
        peer_connections[id(websocket)] = rtc
        rtc.addTrack(CameraVideoTrack(camera))

        await rtc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        answer = await rtc.createAnswer()
        await rtc.setLocalDescription(answer)

        await websocket.send_json({
            "type": "answer",
            "sdp": rtc.localDescription.sdp
        })
    except Exception as e:
        print(f"WebRTC setup error: {e}")


async def add_ice_candidate(pc, candidate_data):
    """Add ICE candidate (runs in background)."""
    try:
        candidate = candidate_from_sdp(candidate_data["candidate"])
        candidate.sdpMid = candidate_data.get("sdpMid")
        candidate.sdpMLineIndex = candidate_data.get("sdpMLineIndex")
        await pc.addIceCandidate(candidate)
    except Exception:
        pass  # Ignore late/invalid candidates


class CameraVideoTrack(MediaStreamTrack):
    """WebRTC video track that streams from RealSense camera."""
    kind = "video"

    def __init__(self, camera_source):
        super().__init__()
        self.camera = camera_source
        self._start = time.time()

    async def recv(self):
        # Calculate presentation timestamp (90kHz timebase)
        pts = int((time.time() - self._start) * 90000)

        # Get frame from camera (thread-safe, non-blocking)
        frame = self.camera.get_color_frame()
        if frame is None:
            # Return black frame if camera not ready
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)

        # Convert BGR (OpenCV) to RGB (av)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = fractions.Fraction(1, 90000)

        # Throttle to ~30fps
        await asyncio.sleep(1/30)
        return video_frame


# FastAPI app
app = FastAPI()
state = {"e": 0, "t": 0.0, "b": 0.0, "s": 0.0, "m": "N"}
active_controller = None
teensy_released = False


def joystick_to_values(x: float, y: float) -> dict:
    """Convert joystick to throttle/brake/steer."""
    return {
        "t": min(max(0, y), MAX_THROTTLE),
        "b": max(0, -y),
        "s": x
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global active_controller, teensy_released
    client_ip = websocket.client.host if websocket.client else "unknown"

    if active_controller is not None:
        print(f"[WS] Rejected {client_ip} - controller already connected")
        await websocket.accept()
        await websocket.send_json({"error": "Another controller is connected"})
        await websocket.close(code=1008)
        return

    await websocket.accept()
    active_controller = websocket
    teensy_released = False  # New connection gets control
    estop = False
    mode = "N"
    print(f"[WS] {client_ip} connected as controller")

    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")

            if msg_type == "control":
                if teensy_released:
                    state["released"] = True
                    await websocket.send_json(state)
                    continue
                x = float(data.get("x", 0))
                y = float(data.get("y", 0))
                v = joystick_to_values(x, y)
                actuator.send_all(estop=estop, throttle=v["t"], mode=mode,
                                  brake=v["b"], steer=v["s"])
                state.update(v)
                state.pop("released", None)

            elif msg_type == "estop":
                estop = data.get("value", False)
                state["e"] = 1 if estop else 0
                if estop:
                    actuator.send_all(estop=True, throttle=0, mode="N", brake=1, steer=0)
                else:
                    actuator.estop(False)

            elif msg_type == "mode":
                mode = data.get("value", "N")
                state["m"] = mode
                if not teensy_released:
                    actuator.set_mode(mode)

            elif msg_type == "teensy_release":
                teensy_released = data.get("value", False)
                if teensy_released:
                    # Safe stop before releasing
                    actuator.send_all(estop=False, throttle=0, mode="N", brake=0, steer=0)
                    print("Teensy released for external control")
                else:
                    print("Teensy control resumed")

            elif msg_type == "offer":
                if not camera or not camera.is_running():
                    await websocket.send_json({"type": "no_camera"})
                else:
                    asyncio.create_task(setup_webrtc(websocket, data["sdp"]))
                continue

            elif msg_type == "candidate":
                pc = peer_connections.get(id(websocket))
                if pc and data.get("candidate"):
                    asyncio.create_task(add_ice_candidate(pc, data))
                continue

            await websocket.send_json(state)

    except WebSocketDisconnect:
        print(f"[WS] {client_ip} disconnected")
    finally:
        # Cleanup WebRTC peer connection
        pc = peer_connections.pop(id(websocket), None)
        if pc:
            await pc.close()

        # Cleanup actuator
        active_controller = None
        actuator.send_all(estop=True, throttle=0, mode="N", brake=1, steer=0)
        print(f"[WS] Controller slot released")


@app.get("/")
async def root():
    return FileResponse("static/index.html")


@app.get("/favicon.ico")
@app.get("/apple-touch-icon.png")
@app.get("/apple-touch-icon-precomposed.png")
async def ignore_icons():
    return Response(status_code=204)


@app.post("/transcribe")
async def transcribe(audio: UploadFile = File(...)):
    """Transcribe audio using Whisper."""
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".webm") as f:
            f.write(await audio.read())
            temp_path = f.name

        segments, _ = whisper_model.transcribe(temp_path, language="en")
        text = " ".join([s.text for s in segments]).strip()
        os.unlink(temp_path)

        return JSONResponse({"text": text})
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


app.mount("/static", StaticFiles(directory="static"), name="static")


@app.middleware("http")
async def no_cache(request, call_next):
    response = await call_next(request)
    if request.url.path.endswith((".js", ".css")):
        response.headers["Cache-Control"] = "no-store"
    return response


if __name__ == "__main__":
    import uvicorn
    print(f"AV Console: https://0.0.0.0:{WEB_PORT}")
    print(f"Teensy: {TEENSY_IP}:{TEENSY_PORT}")
    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT,
                ssl_keyfile="key.pem", ssl_certfile="cert.pem")
