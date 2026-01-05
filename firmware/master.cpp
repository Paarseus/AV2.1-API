// Teensy 4.1 — UDP/Ethernet CAN Master
//
// Network: Static IP 192.168.1.177 | Port 5005 | Watchdog 500ms
//
// Commands (UDP, same as Serial version):
//   E 1|0      -> estop on/off
//   T 0..1     -> throttle
//   M N|D|S|R  -> mode (Neutral/Drive/Sport/Reverse)
//   B 0..1     -> brake
//   S -1..1    -> steer (left -, right +)
//   C          -> center steering
//   A E=0 T=0.5 M=D B=0 S=0.1 C=0  -> all-in-one
//   P          -> request state (returns JSON)
//
// Response (UDP JSON):
//   {"e":0,"t":0.500,"m":"D","b":0.000,"s":0.100,"w":1}
//   w=1 means watchdog active (receiving commands)
//
// CAN frames (11-bit, 100Hz):
//   0x100: [estop][throttle 0-255][mode char]
//   0x200: [center][steer -127..127]
//   0x300: [estop][brake 0-255]

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

using namespace qindesign::network;

// ========== CONFIGURATION ==========
// Network - MP70 router subnet
IPAddress staticIP(192, 168, 13, 177);
IPAddress gateway(192, 168, 13, 31);
IPAddress subnet(255, 255, 255, 0);
const uint16_t UDP_PORT = 5005;

// Safety
const uint32_t WATCHDOG_MS = 500;  // E-stop if no command received

// CAN
const uint32_t CAN_BITRATE = 250000;
const uint32_t CAN_TX_INTERVAL_MS = 10;  // 100Hz

// CAN IDs
const uint32_t THROTTLE_ID = 0x100;
const uint32_t STEER_ID    = 0x200;
const uint32_t BRAKE_ID    = 0x300;

// ========== GLOBALS ==========
EthernetUDP udp;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// Vehicle state
bool  g_estop       = false;
float g_throttle    = 0.0f;
char  g_mode        = 'N';
float g_brake       = 0.0f;
float g_steer       = 0.0f;
bool  g_centerPulse = false;

// Network state
uint32_t g_lastCmd = 0;
bool g_watchdogActive = false;
IPAddress g_lastClient;
uint16_t g_lastClientPort = 0;

// ========== HELPERS ==========
inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
inline float clamp11(float v) { return v < -1 ? -1 : (v > 1 ? 1 : v); }

inline uint8_t toU8(float v) {
    return (uint8_t)(clamp01(v) * 255.0f + 0.5f);
}

inline int8_t toI8(float v) {
    float s = clamp11(v) * 127.0f;
    return (int8_t)(s + (s >= 0 ? 0.5f : -0.5f));
}

// ========== CAN TRANSMISSION ==========
void canSendThrottle() {
    CAN_message_t m;
    m.id = THROTTLE_ID;
    m.len = 3;
    m.buf[0] = g_estop ? 1 : 0;
    m.buf[1] = toU8(g_throttle);
    m.buf[2] = (uint8_t)g_mode;
    Can1.write(m);
}

void canSendBrake() {
    CAN_message_t m;
    m.id = BRAKE_ID;
    m.len = 2;
    m.buf[0] = g_estop ? 1 : 0;
    m.buf[1] = toU8(g_brake);
    Can1.write(m);
}

void canSendSteer() {
    CAN_message_t m;
    m.id = STEER_ID;
    m.len = 2;
    m.buf[0] = g_centerPulse ? 1 : 0;
    m.buf[1] = (uint8_t)toI8(g_steer);
    Can1.write(m);
    g_centerPulse = false;  // One-shot
}

void canSendAll() {
    if (g_estop) {
        // E-stop: brake full, throttle zero, center steering
        g_throttle = 0.0f;
        g_brake = 1.0f;
        g_mode = 'N';
    }
    canSendThrottle();
    canSendBrake();
    canSendSteer();
}

// ========== UDP RESPONSE ==========
void sendState() {
    if (g_lastClientPort == 0) return;

    char buf[80];
    snprintf(buf, sizeof(buf),
        "{\"e\":%d,\"t\":%.3f,\"m\":\"%c\",\"b\":%.3f,\"s\":%.3f,\"w\":%d}",
        g_estop ? 1 : 0,
        g_throttle,
        g_mode,
        g_brake,
        g_steer,
        g_watchdogActive ? 1 : 0
    );

    udp.beginPacket(g_lastClient, g_lastClientPort);
    udp.write(buf);
    udp.endPacket();
}

// ========== COMMAND PARSING ==========
// Parse A-command: "A E=0 T=0.5 M=D B=0 S=0.1 C=0"
void parseAllCommand(char* args) {
    char* save = nullptr;
    char* tok = strtok_r(args, " ,\t", &save);

    while (tok) {
        if (tok[0] && (tok[1] == '=' || tok[1] == ':')) {
            char key = toupper(tok[0]);
            const char* val = tok + 2;

            switch (key) {
                case 'E': g_estop = (atoi(val) != 0); break;
                case 'T': g_throttle = clamp01(atof(val)); break;
                case 'M': {
                    char m = toupper(val[0]);
                    if (m=='N'||m=='D'||m=='S'||m=='R') g_mode = m;
                } break;
                case 'B': g_brake = clamp01(atof(val)); break;
                case 'S': g_steer = clamp11(atof(val)); break;
                case 'C': if (atoi(val) != 0) { g_centerPulse = true; g_steer = 0; } break;
            }
        }
        tok = strtok_r(nullptr, " ,\t", &save);
    }
}

// Parse single command line
void parseCommand(char* line) {
    // Trim leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (!*line) return;

    char cmd = toupper(*line++);
    while (*line == ' ' || *line == '\t') line++;

    switch (cmd) {
        case 'E': g_estop = (atoi(line) != 0); break;
        case 'T': g_throttle = clamp01(atof(line)); break;
        case 'M': {
            char m = toupper(*line);
            if (m=='N'||m=='D'||m=='S'||m=='R') g_mode = m;
        } break;
        case 'B': g_brake = clamp01(atof(line)); break;
        case 'S': g_steer = clamp11(atof(line)); break;
        case 'C': g_centerPulse = true; g_steer = 0; break;
        case 'A': parseAllCommand(line); break;
        case 'P': sendState(); return;  // Don't update watchdog for query
    }

    // Update watchdog and send state response
    g_lastCmd = millis();
    g_watchdogActive = true;
    sendState();
}

// ========== UDP HANDLING ==========
void processUDP() {
    int size = udp.parsePacket();
    if (size <= 0) return;

    // Store sender for response
    g_lastClient = udp.remoteIP();
    g_lastClientPort = udp.remotePort();

    // Read packet
    char buf[128];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;

    buf[len] = '\0';

    // Strip trailing newlines
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) {
        buf[--len] = '\0';
    }

    if (len > 0) {
        parseCommand(buf);
        Serial.print(F("UDP: ")); Serial.println(buf);
    }
}

// ========== WATCHDOG ==========
void checkWatchdog() {
    if (!g_watchdogActive) return;

    if (millis() - g_lastCmd > WATCHDOG_MS) {
        if (!g_estop) {
            g_estop = true;
            g_throttle = 0.0f;
            g_brake = 1.0f;
            Serial.println(F("WATCHDOG TRIGGERED - E-STOP"));
        }
    }
}

// ========== SERIAL DEBUG ==========
void processSerial() {
    static char buf[128];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';

                // Handle local commands
                if (buf[0] == '?') {
                    // Print status to Serial
                    Serial.print(F("State: E=")); Serial.print(g_estop);
                    Serial.print(F(" T=")); Serial.print(g_throttle, 3);
                    Serial.print(F(" M=")); Serial.print(g_mode);
                    Serial.print(F(" B=")); Serial.print(g_brake, 3);
                    Serial.print(F(" S=")); Serial.print(g_steer, 3);
                    Serial.print(F(" W=")); Serial.println(g_watchdogActive);
                } else {
                    parseCommand(buf);
                    Serial.print(F("Serial: ")); Serial.println(buf);
                }
                idx = 0;
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n=== Teensy 4.1 CAN Master (Ethernet) ==="));

    // Ethernet init
    Serial.print(F("Starting Ethernet... "));
    Ethernet.begin(staticIP, subnet, gateway);

    // Wait for link
    uint32_t t0 = millis();
    while (!Ethernet.linkState()) {
        if (millis() - t0 > 5000) {
            Serial.println(F("FAILED - No link"));
            break;
        }
        delay(100);
    }

    if (Ethernet.linkState()) {
        Serial.print(F("OK @ "));
        Serial.print(Ethernet.localIP());
        Serial.print(F(":"));
        Serial.println(UDP_PORT);
    }

    udp.begin(UDP_PORT);

    // CAN init
    Serial.print(F("Starting CAN... "));
    Can1.begin();
    Can1.setBaudRate(CAN_BITRATE);
    Serial.println(F("OK @ 250kbps"));

    Serial.println(F("Commands: E T M B S C A P | Serial: ? for status"));
    Serial.println(F("Ready.\n"));

    g_lastCmd = millis();
}

// ========== MAIN LOOP ==========
void loop() {
    // Process inputs
    processUDP();
    processSerial();

    // Safety check
    checkWatchdog();

    // CAN transmission @ 100Hz
    static uint32_t lastTx = 0;
    if (millis() - lastTx >= CAN_TX_INTERVAL_MS) {
        lastTx = millis();
        canSendAll();
    }
}
