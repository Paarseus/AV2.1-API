#!/usr/bin/env python3
"""
AV Console - Web UI Server (VehicleActuatorUDP version)

WebSocket bridge to Teensy via VehicleActuatorUDP + Whisper voice transcription.
Uses the actuators module for proper state tracking and keepalive.
"""

import sys
import tempfile
import os
from pathlib import Path

# Add FINALE to path for actuator import
FINALE_PATH = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FINALE_PATH))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from faster_whisper import WhisperModel
from actuators import VehicleActuatorUDP

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

# FastAPI app
app = FastAPI()
state = {"e": 0, "t": 0.0, "b": 0.0, "s": 0.0, "m": "N"}
active_controller = None


def joystick_to_values(x: float, y: float) -> dict:
    """Convert joystick to throttle/brake/steer."""
    return {
        "t": min(max(0, y), MAX_THROTTLE),
        "b": max(0, -y),
        "s": x
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global active_controller

    if active_controller is not None:
        await websocket.accept()
        await websocket.send_json({"error": "Another controller is connected"})
        await websocket.close(code=1008)
        return

    await websocket.accept()
    active_controller = websocket
    estop = False
    mode = "N"

    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")

            if msg_type == "control":
                x = float(data.get("x", 0))
                y = float(data.get("y", 0))
                v = joystick_to_values(x, y)
                actuator.send_all(estop=estop, throttle=v["t"], mode=mode,
                                  brake=v["b"], steer=v["s"])
                state.update(v)

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
                actuator.set_mode(mode)

            await websocket.send_json(state)

    except WebSocketDisconnect:
        pass
    finally:
        active_controller = None
        actuator.send_all(estop=True, throttle=0, mode="N", brake=1, steer=0)


@app.get("/")
async def root():
    return FileResponse("static/index.html")


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
