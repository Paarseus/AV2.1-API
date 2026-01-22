// AV Console - WebSocket Client with WebRTC Video

const WS_URL = `wss://${location.host}/ws`;

// DOM elements
const statusEl = document.getElementById('status');
const valuesEl = document.getElementById('values');
const estopBtn = document.getElementById('estop');
const modeButtons = document.querySelectorAll('.modes button');
const micBtn = document.getElementById('mic-btn');
const voiceText = document.getElementById('voice-text');
const remoteVideo = document.getElementById('remote-video');
const videoStatus = document.getElementById('video-status');
const blockedOverlay = document.getElementById('blocked-overlay');
const retryBtn = document.getElementById('retry-btn');
const lockBtn = document.getElementById('lock-btn');
const lockIcon = document.getElementById('lock-icon');
const unlockIcon = document.getElementById('unlock-icon');

// State
let ws = null;
let pc = null;
let estop = false;
let mode = 'N';
let joystickX = 0;
let joystickY = 0;
let sendInterval = null;
let isVoiceActive = false;
let throttleTimeout = null;
let steerTimeout = null;
let mediaRecorder = null;
let audioChunks = [];
let teensyReleased = false;
let isBlocked = false;

// ===== WEBSOCKET =====
function connect() {
    if (ws) {
        ws.onclose = null;
        ws.close();
    }
    if (pc) {
        pc.close();
        pc = null;
    }
    isBlocked = false;

    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        statusEl.textContent = 'Connected';
        statusEl.className = '';
        blockedOverlay.classList.add('hidden');
        teensyReleased = false;
        updateLockUI();
        startSending();
        setTimeout(() => startWebRTC().catch(() => {}), 500);
    };

    ws.onclose = () => {
        stopSending();
        if (pc) { pc.close(); pc = null; }
        remoteVideo.srcObject = null;
        statusEl.textContent = 'Disconnected';
        statusEl.className = 'disconnected';

        // Don't auto-reconnect if blocked - wait for manual retry
        if (!isBlocked) {
            videoStatus.textContent = 'Reconnecting...';
            videoStatus.classList.remove('hidden');
            setTimeout(connect, 1000);
        }
    };

    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);

        // WebRTC signaling
        if (data.type === 'answer') {
            pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: data.sdp }));
            return;
        }
        if (data.type === 'candidate' && data.candidate) {
            pc.addIceCandidate(new RTCIceCandidate({
                candidate: data.candidate,
                sdpMid: data.sdpMid,
                sdpMLineIndex: data.sdpMLineIndex
            }));
            return;
        }
        if (data.type === 'no_camera') {
            videoStatus.textContent = 'Camera unavailable';
            videoStatus.classList.remove('hidden');
            if (pc) { pc.close(); pc = null; }
            return;
        }

        // Blocked by another controller
        if (data.error) {
            isBlocked = true;
            blockedOverlay.classList.remove('hidden');
            stopSending();
            return;
        }

        // Valid state - we have control
        if (data.released) {
            valuesEl.textContent = 'Teensy released — external control';
        } else {
            valuesEl.textContent = `T: ${data.t.toFixed(2)} | S: ${data.s.toFixed(2)} | B: ${data.b.toFixed(2)}`;
        }
    };
}

function send(data) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
    }
}

function startSending() {
    if (sendInterval) return;
    sendInterval = setInterval(() => {
        send({ type: 'control', x: joystickX, y: isVoiceActive ? 0 : joystickY });
    }, 50);
}

function stopSending() {
    if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
}

// ===== WEBRTC =====
async function startWebRTC() {
    videoStatus.textContent = 'Connecting camera...';
    videoStatus.classList.remove('hidden');

    pc = new RTCPeerConnection({ iceServers: [] });

    pc.ontrack = (e) => {
        remoteVideo.srcObject = e.streams[0];
        videoStatus.classList.add('hidden');
    };

    pc.onicecandidate = (e) => {
        if (e.candidate) {
            send({
                type: 'candidate',
                candidate: e.candidate.candidate,
                sdpMid: e.candidate.sdpMid,
                sdpMLineIndex: e.candidate.sdpMLineIndex
            });
        }
    };

    pc.onconnectionstatechange = () => {
        if (pc.connectionState === 'failed') {
            videoStatus.textContent = 'Camera unavailable';
            videoStatus.classList.remove('hidden');
        }
    };

    pc.addTransceiver('video', { direction: 'recvonly' });
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    send({ type: 'offer', sdp: offer.sdp });
}

// ===== JOYSTICK =====
const joystick = nipplejs.create({
    zone: document.getElementById('joystick-zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#1976d2',
    size: 150,
    restOpacity: 0.7
});

joystick.on('move', (evt, data) => {
    const maxDist = 75;
    joystickX = Math.max(-1, Math.min(1, data.vector.x * data.distance / maxDist));
    joystickY = Math.max(-1, Math.min(1, data.vector.y * data.distance / maxDist));
});

joystick.on('end', () => {
    joystickX = 0;
    joystickY = 0;
});

// ===== E-STOP =====
estopBtn.addEventListener('click', () => {
    estop = !estop;
    estopBtn.classList.toggle('active', estop);
    estopBtn.textContent = estop ? 'E-STOP ACTIVE' : 'E-STOP';
    send({ type: 'estop', value: estop });
});

// ===== MODE BUTTONS =====
modeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        modeButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        mode = btn.dataset.mode;
        send({ type: 'mode', value: mode });
    });
});

// ===== VOICE CONTROL =====
function setMode(m) {
    modeButtons.forEach(b => b.classList.remove('active'));
    document.querySelector(`[data-mode="${m}"]`).classList.add('active');
    mode = m;
    send({ type: 'mode', value: m });
}

function setThrottle(value, duration) {
    joystickY = value;
    if (throttleTimeout) clearTimeout(throttleTimeout);
    throttleTimeout = setTimeout(() => { joystickY = 0; }, duration);
}

function setSteering(value, duration) {
    joystickX = value;
    if (steerTimeout) clearTimeout(steerTimeout);
    steerTimeout = setTimeout(() => { joystickX = 0; }, duration);
}

function processVoiceCommand(text) {
    let recognized = true;

    if (text.includes('stop') || text.includes('halt')) {
        estop = true;
        estopBtn.classList.add('active');
        estopBtn.textContent = 'E-STOP ACTIVE';
        send({ type: 'estop', value: true });
    } else if (text.includes('go') || text.includes('forward')) {
        setThrottle(0.4, 2000);
    } else if (text.includes('left')) {
        setSteering(-1, 1000);
    } else if (text.includes('right')) {
        setSteering(1, 1000);
    } else if (text.includes('neutral')) {
        setMode('N');
    } else if (text.includes('drive')) {
        setMode('D');
    } else if (text.includes('reverse') || text.includes('back')) {
        setMode('R');
    } else if (text.includes('sport')) {
        setMode('S');
    } else {
        recognized = false;
        voiceText.textContent = `Unknown: "${text}"`;
    }

    if (recognized) {
        micBtn.className = 'success';
        setTimeout(() => {
            micBtn.className = '';
            voiceText.textContent = 'Hold to speak';
        }, 1500);
    }
}

async function startListening() {
    isVoiceActive = true;
    try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        mediaRecorder = new MediaRecorder(stream);
        audioChunks = [];

        mediaRecorder.ondataavailable = (e) => audioChunks.push(e.data);
        mediaRecorder.onstop = async () => {
            stream.getTracks().forEach(t => t.stop());
            await sendToWhisper(new Blob(audioChunks, { type: 'audio/webm' }));
        };

        mediaRecorder.start();
        micBtn.classList.add('listening');
        voiceText.textContent = 'Listening...';
    } catch (err) {
        voiceText.textContent = 'Mic error: ' + err.message;
        isVoiceActive = false;
    }
}

function stopListening() {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
        micBtn.classList.remove('listening');
        voiceText.textContent = 'Processing...';
    }
}

async function sendToWhisper(blob) {
    try {
        const formData = new FormData();
        formData.append('audio', blob, 'audio.webm');
        const res = await fetch('/transcribe', { method: 'POST', body: formData });
        const data = await res.json();

        if (data.text) {
            const text = data.text.toLowerCase().trim();
            voiceText.textContent = `"${text}"`;
            processVoiceCommand(text);
        } else {
            voiceText.textContent = 'Error: ' + (data.error || 'No text');
        }
    } catch (err) {
        voiceText.textContent = 'Network error';
    }
    isVoiceActive = false;
}

micBtn.addEventListener('mousedown', (e) => { e.stopPropagation(); startListening(); });
micBtn.addEventListener('touchstart', (e) => { e.preventDefault(); e.stopPropagation(); startListening(); });
micBtn.addEventListener('mouseup', (e) => { e.stopPropagation(); stopListening(); });
micBtn.addEventListener('touchend', (e) => { e.stopPropagation(); stopListening(); });
micBtn.addEventListener('mouseleave', stopListening);

// ===== RETRY =====
retryBtn.addEventListener('click', connect);

// ===== TEENSY LOCK =====
lockBtn.addEventListener('click', () => {
    teensyReleased = !teensyReleased;
    send({ type: 'teensy_release', value: teensyReleased });
    updateLockUI();
});

function updateLockUI() {
    lockBtn.classList.toggle('released', teensyReleased);
    lockIcon.classList.toggle('hidden', teensyReleased);
    unlockIcon.classList.toggle('hidden', !teensyReleased);
}

// ===== START =====
connect();
