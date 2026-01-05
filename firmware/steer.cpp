// Teensy 4.1 — Steering over CAN with step-based position tracking
// Encoder used ONLY for homing to center, then steps tracked for position
// ID 0x200 (CMD): Byte0 = center flag (0/1), Byte1 = steer setpoint int8 [-127..127] -> [-1..1]
// ID 0x201 (STATUS): TX basic status; we also accept a request here: Byte0 == 0

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE     250000
#define CAN_ID_CMD      0x200
#define CAN_ID_STATUS   0x201

const uint8_t CMD_STATUS_REQ = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ----------------- Encoder (for homing only) -----------------
const int encoder = A0;
const int CENTER_ENC_LOW  = 900;   // encoder window for center detection
const int CENTER_ENC_HIGH = 910;

// ----------------- Step-based position (CALIBRATED) -----------------
const long STEPS_LEFT  = -5114;   // steps from center to left limit
const long STEPS_RIGHT = 6874;    // steps from center to right limit
const int  STEP_BAND   = 10;      // deadband in steps

long currentSteps = 0;   // 0 = center, - = left, + = right
long targetSteps  = 0;
bool isHomed = false;

// ----------------- Switches & stepper -----------------
const int Rswitch   = 18;   // RIGHT limit switch
const int Lswitch   = 17;   // LEFT limit switch

const int STEP_PIN  = 15;
const int DIR_PIN   = 16;

unsigned int stepHighUs = 50;
unsigned int stepLowUs  = 50;

// ----------------- State -----------------
float steer_setpoint = 0.0f;   // desired [-1..1]
bool needsHoming = true;       // home on startup

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);

  pinMode(Rswitch, INPUT_PULLUP);
  pinMode(Lswitch, INPUT_PULLUP);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  analogReadResolution(10);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("STEER CAN up @ 250k (step-based positioning)");
}

// ----------------- Main Loop -----------------
void loop() {
  // --- CAN RX ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.id == CAN_ID_CMD && msg.len >= 2) {
      bool   centerFlag = (msg.buf[0] != 0);
      int8_t raw_i8     = (int8_t)msg.buf[1];
      float  v          = (float)raw_i8 / 127.0f;

      if (centerFlag) {
        steer_setpoint = 0.0f;
        needsHoming = true;
      } else {
        steer_setpoint = clamp11(v);
      }
    }
    else if (msg.id == CAN_ID_STATUS && msg.len >= 1 && msg.buf[0] == CMD_STATUS_REQ) {
      sendStatusNow();
    }
  }

  // --- Motion control ---
  if (needsHoming) {
    doHome();
  } else {
    steer_update();
  }

  // --- Periodic status ---
  static uint32_t last_status = 0;
  if (millis() - last_status >= 100) {
    last_status = millis();
    sendStatusNow();
  }
}

// ----------------- Helpers -----------------
static inline float clamp11(float v) {
  if (v < -1) return -1;
  if (v >  1) return  1;
  return v;
}

static inline bool rightLimit() { return digitalRead(Rswitch) != LOW; }
static inline bool leftLimit()  { return digitalRead(Lswitch) != LOW; }

inline void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepHighUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepLowUs);
}

// Step right, track position
void stepRight() {
  if (!rightLimit()) {
    digitalWrite(DIR_PIN, LOW);
    pulseStep();
    currentSteps++;
  }
}

// Step left, track position
void stepLeft() {
  if (!leftLimit()) {
    digitalWrite(DIR_PIN, HIGH);
    pulseStep();
    currentSteps--;
  }
}

// ----------------- Homing -----------------
void doHome() {
  int enc = analogRead(encoder);

  if (enc < CENTER_ENC_LOW) {
    if (!rightLimit()) {
      digitalWrite(DIR_PIN, LOW);
      pulseStep();
    }
  } else if (enc > CENTER_ENC_HIGH) {
    if (!leftLimit()) {
      digitalWrite(DIR_PIN, HIGH);
      pulseStep();
    }
  } else {
    currentSteps = 0;
    targetSteps = 0;
    steer_setpoint = 0.0f;
    isHomed = true;
    needsHoming = false;
    Serial.println("Homed to center");
  }
}

// ----------------- Motion Control -----------------
void steer_update() {
  // Map setpoint [-1..1] to target steps
  if (steer_setpoint < 0) {
    targetSteps = (long)(steer_setpoint * (-STEPS_LEFT));  // STEPS_LEFT is negative
  } else {
    targetSteps = (long)(steer_setpoint * STEPS_RIGHT);
  }

  // Clamp to limits
  if (targetSteps < STEPS_LEFT) targetSteps = STEPS_LEFT;
  if (targetSteps > STEPS_RIGHT) targetSteps = STEPS_RIGHT;

  // Move toward target
  long error = targetSteps - currentSteps;

  if (error > STEP_BAND) {
    stepRight();
  } else if (error < -STEP_BAND) {
    stepLeft();
  }
}

// ----------------- Status TX -----------------
void sendStatusNow() {
  // Pack current position as percentage of range
  int8_t pos_pct;
  if (currentSteps < 0) {
    pos_pct = (int8_t)((currentSteps * 127) / (-STEPS_LEFT));
  } else {
    pos_pct = (int8_t)((currentSteps * 127) / STEPS_RIGHT);
  }

  uint8_t flags = 0;
  if (rightLimit()) flags |= 0x01;
  if (leftLimit())  flags |= 0x02;
  if (!isHomed)     flags |= 0x04;

  int8_t sp_i8 = (int8_t)lroundf(clamp11(steer_setpoint) * 127.0f);

  CAN_message_t m;
  m.id  = CAN_ID_STATUS;
  m.len = 8;
  m.buf[0] = (uint8_t)(currentSteps & 0xFF);
  m.buf[1] = (uint8_t)((currentSteps >> 8) & 0xFF);
  m.buf[2] = (uint8_t)((currentSteps >> 16) & 0xFF);
  m.buf[3] = (uint8_t)((currentSteps >> 24) & 0xFF);
  m.buf[4] = pos_pct;              // position as -127..127
  m.buf[5] = flags;                // limit flags + homed flag
  m.buf[6] = sp_i8;                // setpoint as -127..127
  m.buf[7] = isHomed ? 1 : 0;
  Can1.write(m);
}
