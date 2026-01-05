#!/usr/bin/env python3
"""
AV Console - Web UI Server

WebSocket bridge to Teensy via UDP + Whisper voice transcription.
Uses raw UDP socket (standalone, no external dependencies on FINALE modules).
"""

import asyncio
import socket
import tempfile
import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from faster_whisper import WhisperModel

# Configuration
TEENSY_IP = "192.168.13.177"
TEENSY_PORT = 5005
WEB_PORT = 8000
MAX_THROTTLE = 0.4

# Whisper model
print("Loading Whisper model...")
whisper_model = WhisperModel("tiny", device="cpu", compute_type="int8")
print("Whisper ready")

# UDP socket
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setblocking(False)


def send_udp(cmd: str):
    """Send command to Teensy."""
    try:
        udp_sock.sendto(f"{cmd}\n".encode(), (TEENSY_IP, TEENSY_PORT))
    except Exception:
        pass


def joystick_to_command(x: float, y: float, estop: bool, mode: str) -> str:
    """Convert joystick to Teensy command string."""
    throttle = min(max(0, y), MAX_THROTTLE)
    brake = max(0, -y)
    e = 1 if estop else 0
    return f"A E={e} T={throttle:.2f} M={mode} B={brake:.2f} S={x:.2f}"


# FastAPI app
app = FastAPI()
state = {"e": 0, "t": 0.0, "b": 0.0, "s": 0.0, "m": "N"}
active_controller = None


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
                send_udp(joystick_to_command(x, y, estop, mode))
                state["t"] = min(max(0, y), MAX_THROTTLE)
                state["b"] = max(0, -y)
                state["s"] = x

            elif msg_type == "estop":
                estop = data.get("value", False)
                state["e"] = 1 if estop else 0
                if estop:
                    send_udp("A E=1 T=0 M=N B=1 S=0")
                else:
                    send_udp("E 0")

            elif msg_type == "mode":
                mode = data.get("value", "N")
                state["m"] = mode
                send_udp(f"M {mode}")

            await websocket.send_json(state)

    except WebSocketDisconnect:
        pass
    finally:
        active_controller = None
        send_udp("A E=1 T=0 M=N B=1 S=0")


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
