// AV Console - WebSocket Client with WebRTC Video + PID Cruise Control

const WS_URL = `wss://${location.host}/ws`;

// DOM elements
const statusEl = document.getElementById('status');
const estopBtn = document.getElementById('estop');
const modeButtons = document.querySelectorAll('.modes button');
const micBtn = document.getElementById('mic-btn');
const remoteVideo = document.getElementById('remote-video');
const videoStatus = document.getElementById('video-status');
const blockedOverlay = document.getElementById('blocked-overlay');
const retryBtn = document.getElementById('retry-btn');
const lockBtn = document.getElementById('lock-btn');
const lockIcon = document.getElementById('lock-icon');
const unlockIcon = document.getElementById('unlock-icon');

// Video toggle elements
const videoSection = document.getElementById('video-section');
const videoCollapseBtn = document.getElementById('video-collapse-btn');
const videoToggle = document.getElementById('video-toggle');
const videoOverlay = document.getElementById('video-overlay');
const footerSpeed = document.getElementById('footer-speed');

// Telemetry elements
const telemetryPortrait = document.getElementById('telemetry-portrait');
const telemetryLandscape = document.getElementById('telemetry-landscape');

// Cruise control DOM elements
const modeDirectBtn = document.getElementById('mode-direct');
const modeCruiseBtn = document.getElementById('mode-cruise');
const cruisePanel = document.getElementById('cruise-panel');
const cruiseSpeedDisplay = document.getElementById('cruise-speed-display');
const cruiseDecBtn = document.getElementById('cruise-dec');
const cruiseIncBtn = document.getElementById('cruise-inc');
const cruiseEngageBtn = document.getElementById('cruise-engage');
const cruiseCancelBtn = document.getElementById('cruise-cancel');
const cruiseStatusEl = document.getElementById('cruise-status');

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
let videoExpanded = localStorage.getItem('videoExpanded') !== 'false';
let currentSpeed = 0;

// Cruise control state
let controlMode = 'direct';
let cruiseTargetSpeed = 1.0;
let cruiseEngaged = false;
let gpsAvailable = false;

// Initialize video state
updateVideoState();

// ===== VIDEO TOGGLE =====
function updateVideoState() {
    if (videoExpanded) {
        videoSection.classList.add('expanded');
        videoSection.classList.remove('collapsed');
        videoCollapseBtn.textContent = '▲';
        videoToggle.classList.add('hidden');
        footerSpeed.classList.add('hidden');
    } else {
        videoSection.classList.remove('expanded');
        videoSection.classList.add('collapsed');
        videoCollapseBtn.textContent = '▼';
        videoToggle.classList.remove('hidden');
        footerSpeed.classList.remove('hidden');
    }
    localStorage.setItem('videoExpanded', videoExpanded);
}

function toggleVideo() {
    videoExpanded = !videoExpanded;
    updateVideoState();
}

videoCollapseBtn.addEventListener('click', toggleVideo);
videoToggle.addEventListener('click', toggleVideo);

// ===== TELEMETRY UPDATE =====
function updateTelemetry(t, s, b) {
    const text = `T: ${t.toFixed(2)}  S: ${s.toFixed(2)}  B: ${b.toFixed(2)}`;
    if (telemetryPortrait) telemetryPortrait.textContent = text;
    if (telemetryLandscape) telemetryLandscape.textContent = text;
}

function updateSpeed(speed) {
    currentSpeed = speed;
    const speedText = `${speed.toFixed(1)} m/s`;
    if (videoOverlay) videoOverlay.textContent = speedText;
    if (footerSpeed) footerSpeed.textContent = speedText;
}

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
        statusEl.textContent = 'Live';
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
        statusEl.textContent = 'Offline';
        statusEl.className = 'disconnected';

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

        // Valid state - update telemetry
        if (data.released) {
            updateTelemetry(0, 0, 0);
        } else {
            updateTelemetry(data.t, data.s, data.b);
        }

        // Update cruise control state from server
        if (data.cruise) {
            gpsAvailable = data.cruise.gps_available;
            controlMode = data.cruise.mode;
            cruiseEngaged = data.cruise.engaged;

            const speed = data.cruise.current_speed || 0;
            updateSpeed(speed);
            updateCruiseUI();
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
    color: '#2196f3',
    size: 140,
    restOpacity: 0.75
});

joystick.on('move', (evt, data) => {
    const maxDist = 70;
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
    }

    if (recognized) {
        micBtn.className = 'success';
        setTimeout(() => { micBtn.className = ''; }, 1500);
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
    } catch (err) {
        isVoiceActive = false;
    }
}

function stopListening() {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
        micBtn.classList.remove('listening');
    }
}

async function sendToWhisper(blob) {
    try {
        const formData = new FormData();
        formData.append('audio', blob, 'audio.webm');
        const res = await fetch('/transcribe', { method: 'POST', body: formData });
        const data = await res.json();

        if (data.text) {
            processVoiceCommand(data.text.toLowerCase().trim());
        }
    } catch (err) {
        // Silent fail
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

// ===== CRUISE CONTROL =====
function updateCruiseUI() {
    modeDirectBtn.classList.toggle('active', controlMode === 'direct');
    modeCruiseBtn.classList.toggle('active', controlMode === 'cruise');
    modeCruiseBtn.disabled = !gpsAvailable;
    cruisePanel.classList.toggle('visible', controlMode === 'cruise');
    cruiseEngageBtn.classList.toggle('engaged', cruiseEngaged);
    cruiseEngageBtn.textContent = cruiseEngaged ? 'ENGAGED' : 'ENGAGE';
    cruiseSpeedDisplay.textContent = `${cruiseTargetSpeed.toFixed(1)} m/s`;

    if (!gpsAvailable) {
        cruiseStatusEl.textContent = 'GPS Unavailable';
        cruiseStatusEl.className = 'cruise-status gps-error';
    } else if (cruiseEngaged) {
        cruiseStatusEl.textContent = `Engaged at ${cruiseTargetSpeed.toFixed(1)} m/s`;
        cruiseStatusEl.className = 'cruise-status engaged';
    } else {
        cruiseStatusEl.textContent = 'Disengaged';
        cruiseStatusEl.className = 'cruise-status';
    }
}

function setControlMode(newMode) {
    if (newMode === 'cruise' && !gpsAvailable) return;
    controlMode = newMode;
    cruiseEngaged = false;
    send({ type: 'set_control_mode', mode: newMode });
    updateCruiseUI();
}

function adjustCruiseSpeed(delta) {
    cruiseTargetSpeed = Math.max(0, Math.min(2.0, cruiseTargetSpeed + delta));
    cruiseSpeedDisplay.textContent = `${cruiseTargetSpeed.toFixed(1)} m/s`;
    send({ type: 'cruise_speed', speed: cruiseTargetSpeed });
}

function engageCruise() {
    if (controlMode === 'cruise' && gpsAvailable && !cruiseEngaged) {
        send({ type: 'cruise_engage' });
    }
}

function cancelCruise() {
    if (cruiseEngaged) {
        send({ type: 'cruise_cancel' });
    }
}

modeDirectBtn.addEventListener('click', () => setControlMode('direct'));
modeCruiseBtn.addEventListener('click', () => setControlMode('cruise'));
cruiseDecBtn.addEventListener('click', () => adjustCruiseSpeed(-0.1));
cruiseIncBtn.addEventListener('click', () => adjustCruiseSpeed(0.1));
cruiseEngageBtn.addEventListener('click', engageCruise);
cruiseCancelBtn.addEventListener('click', cancelCruise);

updateCruiseUI();

// ===== START =====
connect();
