// Teensy 4.1 + MCP4725 + Digital Mode Lines — Throttle over CAN (0..1)
// ID 0x100 (CMD): Byte0 = E-STOP (0/1), Byte1 = throttle (0..255 -> 0..1), Byte2 = mode char {'N','D','S','R'}
// ID 0x101 (STATUS): we TX status; we also accept a request here: Byte0 == 0

#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <Adafruit_MCP4725.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE      250000
#define THROTTLE_CMD_ID  0x100
#define THROTTLE_STAT_ID 0x101
const uint8_t CMD_STATUS_REQ = 0x00;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ===== Pin Definitions =====
static constexpr uint8_t PIN_MODE_B = 33;
static constexpr uint8_t PIN_MODE_C = 34;
static constexpr uint8_t PIN_MODE_D = 35;

// ===== Types =====
enum TargetMode : uint8_t { TM_NEUTRAL, TM_DRIVE, TM_SPORT, TM_REVERSE };

// ===== Globals =====
Adafruit_MCP4725 dac;
TargetMode currentMode = TM_NEUTRAL;

// MCP4725: 12-bit DAC, output range is 0 to VDD (no internal ref, no gain)
static constexpr float DAC_VDD = 3.3f;

static inline uint16_t v_to_counts(float v) {
  if (v < 0) v = 0;
  if (v > DAC_VDD) v = DAC_VDD;
  return (uint16_t)(v * (4095.0f / DAC_VDD) + 0.5f);
}
static inline float clamp01(float x) { if (x < 0) return 0; if (x > 1) return 1; return x; }

// Throttle control
static constexpr float THROTTLE_IDLE_V = 0.80f;
static constexpr float THROTTLE_MIN_V  = 0.00f;
static constexpr float THROTTLE_MAX_V  = 3.30f;
float throttleV = THROTTLE_IDLE_V;

// ===== Mode line writers (digital pins 33, 34, 35) =====
// Neutral: B=HIGH, C=HIGH, D=HIGH
// Drive:   B=LOW,  C=HIGH, D=HIGH
// Sport:   B=LOW,  C=HIGH, D=LOW
// Reverse: B=HIGH, C=LOW,  D=HIGH

void setNeutralLines() {
  digitalWrite(PIN_MODE_B, HIGH);
  digitalWrite(PIN_MODE_C, HIGH);
  digitalWrite(PIN_MODE_D, HIGH);
}

void setDriveLines() {
  digitalWrite(PIN_MODE_B, LOW);
  digitalWrite(PIN_MODE_C, HIGH);
  digitalWrite(PIN_MODE_D, HIGH);
}

void setSportLines() {
  digitalWrite(PIN_MODE_B, LOW);
  digitalWrite(PIN_MODE_C, HIGH);
  digitalWrite(PIN_MODE_D, LOW);
}

void setReverseLines() {
  digitalWrite(PIN_MODE_B, HIGH);
  digitalWrite(PIN_MODE_C, LOW);
  digitalWrite(PIN_MODE_D, HIGH);
}

// ===== Throttle writer =====
void setThrottleVolts(float v) {
  if (v < THROTTLE_MIN_V) v = THROTTLE_MIN_V;
  if (v > THROTTLE_MAX_V) v = THROTTLE_MAX_V;
  throttleV = v;
  dac.setVoltage(v_to_counts(throttleV), false);
}

void setThrottle01(float x01) {
  x01 = clamp01(x01);
  float v = THROTTLE_IDLE_V + (THROTTLE_MAX_V - THROTTLE_IDLE_V) * x01;
  setThrottleVolts(v);
}

// ===== Mode change =====
void selectMode(TargetMode m) {
  if (m == currentMode) return;

  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  delay(300);

  switch (m) {
    case TM_NEUTRAL: setNeutralLines(); break;
    case TM_DRIVE:   setDriveLines();   break;
    case TM_SPORT:   setSportLines();   break;
    case TM_REVERSE: setReverseLines(); break;
  }
  currentMode = m;
  delay(200);

  setThrottleVolts(throttleV);
}

// ===== Status TX =====
void sendStatusNow() {
  uint16_t a_counts = v_to_counts(throttleV);
  int16_t  v_centi  = (int16_t)lroundf(throttleV * 100.0f);

  CAN_message_t msg;
  msg.id  = THROTTLE_STAT_ID;
  msg.len = 8;
  msg.buf[0] = (uint8_t)(a_counts >> 8);
  msg.buf[1] = (uint8_t)(a_counts & 0xFF);
  msg.buf[2] = (uint8_t)currentMode;       // 0=N,1=D,2=S,3=R
  msg.buf[3] = 0;                          // reserved
  msg.buf[4] = (uint8_t)(v_centi & 0xFF);  // V*100 (LE)
  msg.buf[5] = (uint8_t)((v_centi >> 8) & 0xFF);
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can1.write(msg);
}

// ===== Setup / Loop =====
uint32_t last_cmd_ms = 0;
const uint32_t WATCHDOG_MS = 200;

void setup() {
  Serial.begin(115200);

  // Setup digital pins for mode control
  pinMode(PIN_MODE_B, OUTPUT);
  pinMode(PIN_MODE_C, OUTPUT);
  pinMode(PIN_MODE_D, OUTPUT);

  Wire.begin();
  if (!dac.begin(0x62)) {
    if (!dac.begin(0x60)) {
      Serial.println("ERROR: MCP4725 not found. Check wiring.");
      while (1) delay(250);
    }
  }

  // Boot safe in Neutral + idle
  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  currentMode = TM_NEUTRAL;

  // CAN up
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
}

void loop() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Status request
    if (msg.id == THROTTLE_STAT_ID && msg.len >= 1 && msg.buf[0] == CMD_STATUS_REQ) {
      sendStatusNow();
      continue;
    }

    // Throttle command
    if (msg.id == THROTTLE_CMD_ID && msg.len >= 3) {
      bool estop = (msg.buf[0] != 0);
      float thr01 = (float)msg.buf[1] / 255.0f;
      char modeCh = (char)msg.buf[2];

      if (estop) {
        selectMode(TM_NEUTRAL);
        setThrottleVolts(0.0f);
      } else {
        TargetMode m = currentMode;
        if      (modeCh == 'N') m = TM_NEUTRAL;
        else if (modeCh == 'D') m = TM_DRIVE;
        else if (modeCh == 'S') m = TM_SPORT;
        else if (modeCh == 'R') m = TM_REVERSE;
        if (m != currentMode) selectMode(m);

        setThrottle01(thr01);
      }
      last_cmd_ms = millis();
    }
  }

  // Watchdog: if commands stop arriving, fall back to Neutral + Idle
  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    if (currentMode != TM_NEUTRAL) selectMode(TM_NEUTRAL);
    setThrottleVolts(THROTTLE_IDLE_V);
  }

  // Periodic status at 10 Hz
  static uint32_t last_status = 0;
  uint32_t now = millis();
  if (now - last_status >= 100) {
    last_status = now;
    sendStatusNow();
  }
}
