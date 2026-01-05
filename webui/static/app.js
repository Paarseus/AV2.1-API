// AV Console - Joystick WebSocket Client

const WS_URL = `wss://${location.host}/ws`;

// Cache DOM elements
const statusEl = document.getElementById('status');
const valuesEl = document.getElementById('values');
const estopBtn = document.getElementById('estop');
const modeButtons = document.querySelectorAll('.modes button');
const micBtn = document.getElementById('mic-btn');
const voiceText = document.getElementById('voice-text');

let ws = null;
let estop = false;
let mode = 'D';
let joystickX = 0;
let joystickY = 0;
let sendInterval = null;
let isVoiceActive = false;
let voiceTimeout = null;
let mediaRecorder = null;
let audioChunks = [];

// ===== WEBSOCKET =====
function connect() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        statusEl.textContent = 'Connected';
        statusEl.className = '';
        startSending();
    };

    ws.onclose = () => {
        statusEl.textContent = 'Disconnected';
        statusEl.className = 'disconnected';
        stopSending();
        setTimeout(connect, 1000);
    };

    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);
        if (data.error) {
            statusEl.textContent = data.error;
            statusEl.className = 'disconnected';
            stopSending();
            return;
        }
        valuesEl.textContent = `T: ${data.t.toFixed(2)} | S: ${data.s.toFixed(2)} | B: ${data.b.toFixed(2)}`;
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
        const y = isVoiceActive ? 0 : joystickY;
        send({ type: 'control', x: joystickX, y: y });
    }, 50);
}

function stopSending() {
    if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
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
    if (voiceTimeout) clearTimeout(voiceTimeout);
    voiceTimeout = setTimeout(() => { joystickY = 0; }, duration);
}

function setSteering(value, duration) {
    joystickX = value;
    if (voiceTimeout) clearTimeout(voiceTimeout);
    voiceTimeout = setTimeout(() => { joystickX = 0; }, duration);
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
            const blob = new Blob(audioChunks, { type: 'audio/webm' });
            await sendToWhisper(blob);
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

// ===== START =====
connect();
