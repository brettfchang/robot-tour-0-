// Simple Sequence Control with Velocity (BNO085 version)
// Uses minimal BNO085 driver for game rotation vector only
// Fits on Arduino Uno with ~200 bytes RAM for IMU instead of ~3000
//
// Commands:
//   f = forward (DRIVE_DISTANCE mm)
//   b = backward
//   l = turn left 90°
//   r = turn right 90°
//
// Button 4 runs the sequence
//
// BNO085 Wiring:
//   VCC -> 3.3V (or 5V if board has regulator)
//   GND -> GND
//   SDA -> A4
//   SCL -> A5
//   RST -> Pin 8

#include <Wire.h>

// ============== HARDWARE MAPPING ==============
// Motor A = LEFT, Motor B = RIGHT
// Enc0 (A0,A1) = RIGHT wheel, +ve forward
// Enc1 (A2,A3) = LEFT wheel, -ve forward (negated in ISR)

// Motor pins
const int PWMA_PIN = 11;
const int AIN1_PIN = 10;
const int AIN2_PIN = 12;
const int PWMB_PIN = 5;
const int BIN1_PIN = 7;
const int BIN2_PIN = 6;
const int STBY_PIN = 9;

// Button
const int BUTTON_PIN = 4;

// BNO085
const int BNO085_RST_PIN = 8;
const uint8_t BNO085_ADDR = 0x4A;

// ============== CONFIGURATION ==============

const float DRIVE_DISTANCE = 500.0;
const float WHEEL_DIAMETER_MM = 73.025;
const int COUNTS_PER_REV = 900;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

const float TARGET_VELOCITY = 200.0;
const float TURN_VELOCITY = 100.0;
const float VELOCITY_RAMP = 150.0;

const float KFF = 0.8;
const int SAMPLE_INTERVAL_MS = 10;

const float HEADING_Kp = 3.0;
const float TURN_Kp = 2.0;

const float HEADING_TOLERANCE = 2.0;
const int MIN_PWM = 50;
const int MAX_PWM = 255;

const char* SEQUENCE = "frfrfrfr";

// ============== STATE ==============

volatile long encLeft = 0;
volatile long encRight = 0;

float heading = 0;
float headingOffset = 0;
int targetHeading = 0;

// ============== MINIMAL BNO085 DRIVER ==============
// Only supports game rotation vector to save RAM

#define BNO_CHANNEL_CONTROL 2
#define BNO_CHANNEL_REPORTS 3
#define GAME_ROTATION_VECTOR 0x08
#define SET_FEATURE_CMD 0xFD

uint8_t bnoSeqNum[6] = {0};  // Sequence numbers per channel
uint8_t shtpBuffer[48];      // Shared buffer for TX/RX
float quat[4];               // i, j, k, real

bool bnoSendPacket(uint8_t channel, uint8_t len) {
  uint8_t totalLen = len + 4;

  Wire.beginTransmission(BNO085_ADDR);
  Wire.write(totalLen & 0xFF);
  Wire.write(totalLen >> 8);
  Wire.write(channel);
  Wire.write(bnoSeqNum[channel]++);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(shtpBuffer[i]);
  }
  return Wire.endTransmission() == 0;
}

int16_t bnoReceivePacket() {
  // Read header + payload in one request (up to 32 bytes - Wire buffer limit)
  uint8_t bytesRead = Wire.requestFrom(BNO085_ADDR, (uint8_t)32);

  if (bytesRead == 0 || Wire.available() < 4) {
    return -1;
  }

  uint8_t lenLSB = Wire.read();
  uint8_t lenMSB = Wire.read();
  uint8_t channel = Wire.read();
  uint8_t seqNum = Wire.read();
  (void)seqNum;

  uint16_t totalLen = ((uint16_t)(lenMSB & 0x7F) << 8) | lenLSB;
  if (totalLen == 0 || totalLen == 0x7FFF) return -1;

  uint16_t dataLen = totalLen - 4;
  if (dataLen > sizeof(shtpBuffer)) dataLen = sizeof(shtpBuffer);

  // Read remaining payload from buffer
  uint8_t i = 0;
  while (Wire.available() && i < dataLen) {
    shtpBuffer[i++] = Wire.read();
  }

  // Store channel in high byte, length in low byte
  return (channel << 8) | dataLen;
}

void bnoEnableGameRotation(uint16_t intervalMs) {
  uint32_t intervalUs = (uint32_t)intervalMs * 1000;

  shtpBuffer[0] = SET_FEATURE_CMD;
  shtpBuffer[1] = GAME_ROTATION_VECTOR;
  shtpBuffer[2] = 0;  // Feature flags
  shtpBuffer[3] = 0;  // Change sensitivity LSB
  shtpBuffer[4] = 0;  // Change sensitivity MSB
  shtpBuffer[5] = intervalUs & 0xFF;
  shtpBuffer[6] = (intervalUs >> 8) & 0xFF;
  shtpBuffer[7] = (intervalUs >> 16) & 0xFF;
  shtpBuffer[8] = (intervalUs >> 24) & 0xFF;
  shtpBuffer[9] = 0;  // Batch interval
  shtpBuffer[10] = 0;
  shtpBuffer[11] = 0;
  shtpBuffer[12] = 0;
  shtpBuffer[13] = 0;  // Sensor-specific config
  shtpBuffer[14] = 0;
  shtpBuffer[15] = 0;
  shtpBuffer[16] = 0;

  bnoSendPacket(BNO_CHANNEL_CONTROL, 17);
}

bool bnoUpdate() {
  int16_t result = bnoReceivePacket();
  if (result < 0) return false;

  uint8_t channel = result >> 8;
  uint8_t dataLen = result & 0xFF;

  // Only parse input reports on channel 3
  if (channel != BNO_CHANNEL_REPORTS) return false;

  // Search for game rotation vector report in the packet
  // Reports may be preceded by timestamp reference (0xFB)
  uint8_t offset = 0;
  while (offset < dataLen) {
    uint8_t reportId = shtpBuffer[offset];

    if (reportId == GAME_ROTATION_VECTOR) {
      // Found it! Parse quaternion at offset+4 (skip report header)
      // Format: reportId(1), seq(1), status(1), delay(2), i(2), j(2), k(2), real(2)
      uint8_t base = offset + 5;  // Skip to quaternion data

      int16_t qi = (int16_t)((shtpBuffer[base + 1] << 8) | shtpBuffer[base]);
      int16_t qj = (int16_t)((shtpBuffer[base + 3] << 8) | shtpBuffer[base + 2]);
      int16_t qk = (int16_t)((shtpBuffer[base + 5] << 8) | shtpBuffer[base + 4]);
      int16_t qr = (int16_t)((shtpBuffer[base + 7] << 8) | shtpBuffer[base + 6]);

      const float Q14_SCALE = 1.0f / 16384.0f;
      quat[0] = qi * Q14_SCALE;
      quat[1] = qj * Q14_SCALE;
      quat[2] = qk * Q14_SCALE;
      quat[3] = qr * Q14_SCALE;

      return true;
    }
    else if (reportId == 0xFB) {
      // Base timestamp reference - 5 bytes, skip it
      offset += 5;
    }
    else if (reportId == 0xFC) {
      // Timestamp rebase - 5 bytes, skip it
      offset += 5;
    }
    else {
      // Unknown report, try to skip (assume minimum 1 byte)
      offset++;
    }
  }

  return false;
}

bool bnoInit() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C

  pinMode(BNO085_RST_PIN, OUTPUT);

  // Hardware reset
  digitalWrite(BNO085_RST_PIN, LOW);
  delay(15);
  digitalWrite(BNO085_RST_PIN, HIGH);
  delay(500);  // Wait for boot - BNO085 needs time

  // Flush any pending data
  for (int i = 0; i < 20; i++) {
    bnoReceivePacket();
    delay(20);
  }

  // Enable game rotation vector at 100Hz
  bnoEnableGameRotation(10);
  delay(100);

  // Wait for first valid reading
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (bnoUpdate()) return true;
    delay(10);
  }

  return false;
}

// ============== ENCODER ISR ==============

volatile uint8_t lastPinState = 0;

const int8_t QUAD_TABLE[16] PROGMEM = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

ISR(PCINT1_vect) {
  uint8_t currentState = PINC;

  uint8_t e0OldA = (lastPinState >> 0) & 1;
  uint8_t e0OldB = (lastPinState >> 1) & 1;
  uint8_t e0NewA = (currentState >> 0) & 1;
  uint8_t e0NewB = (currentState >> 1) & 1;
  uint8_t e0Idx = (e0OldA << 3) | (e0OldB << 2) | (e0NewA << 1) | e0NewB;
  encRight += pgm_read_byte(&QUAD_TABLE[e0Idx]);

  uint8_t e1OldA = (lastPinState >> 2) & 1;
  uint8_t e1OldB = (lastPinState >> 3) & 1;
  uint8_t e1NewA = (currentState >> 2) & 1;
  uint8_t e1NewB = (currentState >> 3) & 1;
  uint8_t e1Idx = (e1OldA << 3) | (e1OldB << 2) | (e1NewA << 1) | e1NewB;
  encLeft -= pgm_read_byte(&QUAD_TABLE[e1Idx]);

  lastPinState = currentState;
}

void setupEncoders() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  lastPinState = PINC;
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
}

// ============== IMU WRAPPER ==============

void setupIMU() {
  Serial.println(F("Init BNO085..."));

  if (!bnoInit()) {
    Serial.println(F("BNO085 failed!"));
    while (1) delay(10);
  }

  Serial.println(F("BNO085 OK"));

  // Get initial heading as offset
  updateIMU();
  headingOffset = heading;
  heading = 0;
}

void updateIMU() {
  if (!bnoUpdate()) return;

  // Convert quaternion to yaw
  float qi = quat[0];
  float qj = quat[1];
  float qk = quat[2];
  float qr = quat[3];

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  float yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;

  heading = yaw - headingOffset;

  while (heading > 180) heading -= 360;
  while (heading < -180) heading += 360;
}

// ============== MOTORS ==============

void setupMotors() {
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void drive(int pwmL, int pwmR) {
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  if (pwmL < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  }

  if (pwmR < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  } else {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  }

  int absL = abs(pwmL);
  int absR = abs(pwmR);
  if (absL > 0 && absL < MIN_PWM) absL = MIN_PWM;
  if (absR > 0 && absR < MIN_PWM) absR = MIN_PWM;

  analogWrite(PWMA_PIN, absL);
  analogWrite(PWMB_PIN, absR);
}

void stop() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
}

// ============== MOVEMENT ==============

float getHeadingError() {
  float error = targetHeading - heading;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

void driveForward(float distanceMm) {
  Serial.print(F("Forward "));
  Serial.print((int)distanceMm);
  Serial.println(F(" mm"));

  noInterrupts();
  long startLeft = encLeft;
  long startRight = encRight;
  long lastLeft = startLeft;
  long lastRight = startRight;
  interrupts();

  unsigned long lastTime = millis();
  float currentTargetVel = 0;

  while (true) {
    updateIMU();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    float distLeft = (left - startLeft) * MM_PER_COUNT;
    float distRight = (right - startRight) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0f;
    float distRemaining = distanceMm - distTraveled;

    float velLeft = (left - lastLeft) * MM_PER_COUNT / dt;
    float velRight = (right - lastRight) * MM_PER_COUNT / dt;
    lastLeft = left;
    lastRight = right;

    if (distRemaining <= 0) {
      stop();
      Serial.println(F("Done"));
      return;
    }

    float maxVel = TARGET_VELOCITY;
    if (distRemaining < 100.0f) {
      maxVel = TARGET_VELOCITY * (distRemaining / 100.0f);
      if (maxVel < 30.0f) maxVel = 30.0f;
    }

    if (currentTargetVel < maxVel) {
      currentTargetVel += VELOCITY_RAMP * dt;
      if (currentTargetVel > maxVel) currentTargetVel = maxVel;
    } else if (currentTargetVel > maxVel) {
      currentTargetVel -= VELOCITY_RAMP * dt;
      if (currentTargetVel < maxVel) currentTargetVel = maxVel;
    }

    float headingError = getHeadingError();
    float velCorrection = headingError * HEADING_Kp;

    float targetVelLeft = currentTargetVel + velCorrection;
    float targetVelRight = currentTargetVel - velCorrection;

    float pwmL = KFF * targetVelLeft;
    float pwmR = KFF * targetVelRight;

    float velErrorL = targetVelLeft - velLeft;
    float velErrorR = targetVelRight - velRight;
    pwmL += velErrorL * 0.25f;
    pwmR += velErrorR * 0.25f;

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    // Output for visualizer
    Serial.print(F("hdg:"));
    Serial.print(heading, 1);
    Serial.print(F(" tgtHdg:"));
    Serial.print(targetHeading);
    Serial.print(F(" tgtVelL:"));
    Serial.print(targetVelLeft, 1);
    Serial.print(F(" tgtVelR:"));
    Serial.print(targetVelRight, 1);
    Serial.print(F(" velL:"));
    Serial.print(velLeft, 1);
    Serial.print(F(" velR:"));
    Serial.print(velRight, 1);
    Serial.print(F(" pwmL:"));
    Serial.print((int)pwmL);
    Serial.print(F(" pwmR:"));
    Serial.print((int)pwmR);
    Serial.print(F(" ffL:"));
    Serial.print(KFF * targetVelLeft, 1);
    Serial.print(F(" ffR:"));
    Serial.println(KFF * targetVelRight, 1);

    drive((int)pwmL, (int)pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void driveBackward(float distanceMm) {
  Serial.print(F("Backward "));
  Serial.print((int)distanceMm);
  Serial.println(F(" mm"));

  noInterrupts();
  long startLeft = encLeft;
  long startRight = encRight;
  long lastLeft = startLeft;
  long lastRight = startRight;
  interrupts();

  unsigned long lastTime = millis();
  float currentTargetVel = 0;

  while (true) {
    updateIMU();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    float distLeft = (startLeft - left) * MM_PER_COUNT;
    float distRight = (startRight - right) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0f;
    float distRemaining = distanceMm - distTraveled;

    float velLeft = (left - lastLeft) * MM_PER_COUNT / dt;
    float velRight = (right - lastRight) * MM_PER_COUNT / dt;
    lastLeft = left;
    lastRight = right;

    if (distRemaining <= 0) {
      stop();
      Serial.println(F("Done"));
      return;
    }

    float maxVel = TARGET_VELOCITY;
    if (distRemaining < 100.0f) {
      maxVel = TARGET_VELOCITY * (distRemaining / 100.0f);
      if (maxVel < 30.0f) maxVel = 30.0f;
    }

    if (currentTargetVel < maxVel) {
      currentTargetVel += VELOCITY_RAMP * dt;
      if (currentTargetVel > maxVel) currentTargetVel = maxVel;
    } else if (currentTargetVel > maxVel) {
      currentTargetVel -= VELOCITY_RAMP * dt;
      if (currentTargetVel < maxVel) currentTargetVel = maxVel;
    }

    float headingError = getHeadingError();
    float velCorrection = headingError * HEADING_Kp;

    float targetVelLeft = -currentTargetVel + velCorrection;
    float targetVelRight = -currentTargetVel - velCorrection;

    float pwmL = KFF * targetVelLeft;
    float pwmR = KFF * targetVelRight;

    float velErrorL = targetVelLeft - velLeft;
    float velErrorR = targetVelRight - velRight;
    pwmL += velErrorL * 0.25f;
    pwmR += velErrorR * 0.25f;

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    // Output for visualizer
    Serial.print(F("hdg:"));
    Serial.print(heading, 1);
    Serial.print(F(" tgtHdg:"));
    Serial.print(targetHeading);
    Serial.print(F(" tgtVelL:"));
    Serial.print(targetVelLeft, 1);
    Serial.print(F(" tgtVelR:"));
    Serial.print(targetVelRight, 1);
    Serial.print(F(" velL:"));
    Serial.print(velLeft, 1);
    Serial.print(F(" velR:"));
    Serial.print(velRight, 1);
    Serial.print(F(" pwmL:"));
    Serial.print((int)pwmL);
    Serial.print(F(" pwmR:"));
    Serial.print((int)pwmR);
    Serial.print(F(" ffL:"));
    Serial.print(KFF * targetVelLeft, 1);
    Serial.print(F(" ffR:"));
    Serial.println(KFF * targetVelRight, 1);

    drive((int)pwmL, (int)pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void turnLeft() {
  Serial.println(F("Turn left 90"));
  targetHeading += 90;
  if (targetHeading >= 360) targetHeading -= 360;

  while (true) {
    updateIMU();

    float error = getHeadingError();

    if (abs(error) < HEADING_TOLERANCE) {
      stop();
      delay(100);
      updateIMU();
      if (abs(getHeadingError()) < HEADING_TOLERANCE) {
        Serial.println(F("Done"));
        return;
      }
    }

    float targetTurnVel = error * TURN_Kp;
    targetTurnVel = constrain(targetTurnVel, -TURN_VELOCITY, TURN_VELOCITY);

    float pwmL = KFF * targetTurnVel;
    float pwmR = KFF * (-targetTurnVel);

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    // Output for visualizer
    Serial.print(F("hdg:"));
    Serial.print(heading, 1);
    Serial.print(F(" tgtHdg:"));
    Serial.print(targetHeading);
    Serial.print(F(" turnVel:"));
    Serial.print(targetTurnVel, 1);
    Serial.print(F(" pwmL:"));
    Serial.print((int)pwmL);
    Serial.print(F(" pwmR:"));
    Serial.println((int)pwmR);

    drive((int)pwmL, (int)pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void turnRight() {
  Serial.println(F("Turn right 90"));
  targetHeading -= 90;
  if (targetHeading < 0) targetHeading += 360;

  while (true) {
    updateIMU();

    float error = getHeadingError();

    if (abs(error) < HEADING_TOLERANCE) {
      stop();
      delay(100);
      updateIMU();
      if (abs(getHeadingError()) < HEADING_TOLERANCE) {
        Serial.println(F("Done"));
        return;
      }
    }

    float targetTurnVel = error * TURN_Kp;
    targetTurnVel = constrain(targetTurnVel, -TURN_VELOCITY, TURN_VELOCITY);

    float pwmL = KFF * targetTurnVel;
    float pwmR = KFF * (-targetTurnVel);

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    // Output for visualizer
    Serial.print(F("hdg:"));
    Serial.print(heading, 1);
    Serial.print(F(" tgtHdg:"));
    Serial.print(targetHeading);
    Serial.print(F(" turnVel:"));
    Serial.print(targetTurnVel, 1);
    Serial.print(F(" pwmL:"));
    Serial.print((int)pwmL);
    Serial.print(F(" pwmR:"));
    Serial.println((int)pwmR);

    drive((int)pwmL, (int)pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

// ============== SEQUENCE RUNNER ==============

void runSequence(const char* seq) {
  Serial.print(F("Seq: "));
  Serial.println(seq);

  updateIMU();
  headingOffset = heading + headingOffset;
  heading = 0;
  targetHeading = 0;

  for (int i = 0; seq[i] != '\0'; i++) {
    char cmd = seq[i];

    switch (cmd) {
      case 'f': case 'F': driveForward(DRIVE_DISTANCE); break;
      case 'b': case 'B': driveBackward(DRIVE_DISTANCE); break;
      case 'l': case 'L': turnLeft(); break;
      case 'r': case 'R': turnRight(); break;
    }

    delay(200);
  }

  Serial.println(F("Done!"));
}

// ============== MAIN ==============

int lastButtonVal = HIGH;

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupEncoders();
  setupIMU();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  stop();

  Serial.println(F("Ready - btn 4"));
}

void loop() {
  int buttonVal = digitalRead(BUTTON_PIN);

  if (lastButtonVal == HIGH && buttonVal == LOW) {
    delay(50);
    runSequence(SEQUENCE);
  }

  lastButtonVal = buttonVal;
}
