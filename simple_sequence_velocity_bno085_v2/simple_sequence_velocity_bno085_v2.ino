// Simple Sequence Control with Adaptive Velocity (BNO085 version)
// Drive sequences like "frfl" (forward, right, forward, left)
// Uses adaptive feedforward that learns PWM-per-velocity coefficient
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

// Distance to drive for 'f' and 'b' commands (mm)
const float DRIVE_DISTANCE = 500.0;

// Robot measurements
const float WHEEL_DIAMETER_MM = 73.025;
const int COUNTS_PER_REV = 900;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// Velocity control
const float TARGET_VELOCITY = 200.0;   // mm/s for straights
const float TURN_VELOCITY = 100.0;     // mm/s for turns
const float VELOCITY_RAMP = 150.0;     // mm/s^2 acceleration limit

// Adaptive feedforward
const float INITIAL_KFF = 0.8;         // Initial PWM per mm/s
const float ADAPT_RATE = 0.02;         // How fast coefficient adapts (per sample)
const int ADAPT_WINDOW_MS = 1000;      // Window for smoothing (1 second)
const int SAMPLE_INTERVAL_MS = 10;     // Control loop interval
const int SAMPLES_IN_WINDOW = ADAPT_WINDOW_MS / SAMPLE_INTERVAL_MS;  // 100 samples

// Heading control
const float HEADING_Kp = 3.0;          // Heading correction during straights
const float TURN_Kp = 2.0;             // Turn speed control

// Tolerances
const float HEADING_TOLERANCE = 2.0;   // degrees
const float DISTANCE_TOLERANCE = 5.0;  // mm
const int MIN_PWM = 50;
const int MAX_PWM = 255;

// The sequence to run (edit this!)
const char* SEQUENCE = "frfrfrfr";  // Square: forward, right, x4

// ============== STATE ==============

volatile long encLeft = 0;
volatile long encRight = 0;

float heading = 0;          // Current heading from IMU (degrees)
int targetHeading = 0;      // Target heading (0, 90, 180, 270)
float headingOffset = 0;    // For zeroing heading

// Adaptive feedforward state
float kffLeft = INITIAL_KFF;   // Learned coefficient for left wheel
float kffRight = INITIAL_KFF;  // Learned coefficient for right wheel

// Circular buffers for smoothing (stores PWM/velocity ratios)
float ratioHistoryLeft[100];
float ratioHistoryRight[100];
int historyIndex = 0;
int historyCount = 0;

// ============== MINIMAL BNO085 DRIVER ==============

#define BNO_CHANNEL_CONTROL 2
#define BNO_CHANNEL_REPORTS 3
#define GAME_ROTATION_VECTOR 0x08
#define SET_FEATURE_CMD 0xFD

uint8_t bnoSeqNum[6] = {0};
uint8_t shtpBuffer[48];
float quat[4];  // i, j, k, real

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
  // Read header + payload in one request (up to 32 bytes)
  uint8_t bytesRead = Wire.requestFrom(BNO085_ADDR, (uint8_t)32);

  if (bytesRead == 0 || Wire.available() < 4) return -1;

  uint8_t lenLSB = Wire.read();
  uint8_t lenMSB = Wire.read();
  uint8_t channel = Wire.read();
  Wire.read();  // sequence, discard

  uint16_t totalLen = ((uint16_t)(lenMSB & 0x7F) << 8) | lenLSB;
  if (totalLen == 0 || totalLen == 0x7FFF) return -1;

  uint16_t dataLen = totalLen - 4;
  if (dataLen > sizeof(shtpBuffer)) dataLen = sizeof(shtpBuffer);

  uint8_t i = 0;
  while (Wire.available() && i < dataLen) {
    shtpBuffer[i++] = Wire.read();
  }

  return (channel << 8) | i;
}

void bnoEnableGameRotation(uint16_t intervalMs) {
  uint32_t intervalUs = (uint32_t)intervalMs * 1000;

  shtpBuffer[0] = SET_FEATURE_CMD;
  shtpBuffer[1] = GAME_ROTATION_VECTOR;
  shtpBuffer[2] = 0;
  shtpBuffer[3] = 0;
  shtpBuffer[4] = 0;
  shtpBuffer[5] = intervalUs & 0xFF;
  shtpBuffer[6] = (intervalUs >> 8) & 0xFF;
  shtpBuffer[7] = (intervalUs >> 16) & 0xFF;
  shtpBuffer[8] = (intervalUs >> 24) & 0xFF;
  shtpBuffer[9] = 0;
  shtpBuffer[10] = 0;
  shtpBuffer[11] = 0;
  shtpBuffer[12] = 0;
  shtpBuffer[13] = 0;
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

  if (channel != BNO_CHANNEL_REPORTS) return false;

  // Search for game rotation vector in packet
  uint8_t offset = 0;
  while (offset < dataLen) {
    uint8_t reportId = shtpBuffer[offset];

    if (reportId == GAME_ROTATION_VECTOR) {
      uint8_t base = offset + 5;

      // Ensure we have enough data
      if (base + 8 > dataLen) {
        offset++;
        continue;
      }

      // Big-endian: MSB first
      int16_t qi = (int16_t)((shtpBuffer[base] << 8) | shtpBuffer[base + 1]);
      int16_t qj = (int16_t)((shtpBuffer[base + 2] << 8) | shtpBuffer[base + 3]);
      int16_t qk = (int16_t)((shtpBuffer[base + 4] << 8) | shtpBuffer[base + 5]);
      int16_t qr = (int16_t)((shtpBuffer[base + 6] << 8) | shtpBuffer[base + 7]);

      const float Q14_SCALE = 1.0f / 16384.0f;
      float ti = qi * Q14_SCALE;
      float tj = qj * Q14_SCALE;
      float tk = qk * Q14_SCALE;
      float tr = qr * Q14_SCALE;

      // Validate: quaternion components must be between -1 and 1
      if (ti >= -1.0f && ti <= 1.0f && tj >= -1.0f && tj <= 1.0f &&
          tk >= -1.0f && tk <= 1.0f && tr >= -1.0f && tr <= 1.0f) {
        quat[0] = ti;
        quat[1] = tj;
        quat[2] = tk;
        quat[3] = tr;
        return true;
      }
      offset += 13;  // Skip past this report even if rejected
    }
    else if (reportId == 0xFB || reportId == 0xFC) {
      offset += 5;  // Timestamp reference
    }
    else {
      offset++;
    }
  }

  return false;
}

bool bnoInit() {
  Wire.begin();
  Wire.setClock(400000);

  pinMode(BNO085_RST_PIN, OUTPUT);

  digitalWrite(BNO085_RST_PIN, LOW);
  delay(15);
  digitalWrite(BNO085_RST_PIN, HIGH);
  delay(500);

  for (int i = 0; i < 20; i++) {
    bnoReceivePacket();
    delay(20);
  }

  bnoEnableGameRotation(10);
  delay(100);

  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (bnoUpdate()) return true;
    delay(10);
  }

  return false;
}

// ============== ENCODER ISR ==============

volatile uint8_t lastPinState = 0;

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

ISR(PCINT1_vect) {
  uint8_t currentState = PINC;

  // Enc0 (A0,A1) = RIGHT wheel
  uint8_t e0OldA = (lastPinState >> 0) & 1;
  uint8_t e0OldB = (lastPinState >> 1) & 1;
  uint8_t e0NewA = (currentState >> 0) & 1;
  uint8_t e0NewB = (currentState >> 1) & 1;
  uint8_t e0Idx = (e0OldA << 3) | (e0OldB << 2) | (e0NewA << 1) | e0NewB;
  encRight += QUAD_TABLE[e0Idx];

  // Enc1 (A2,A3) = LEFT wheel (negated)
  uint8_t e1OldA = (lastPinState >> 2) & 1;
  uint8_t e1OldB = (lastPinState >> 3) & 1;
  uint8_t e1NewA = (currentState >> 2) & 1;
  uint8_t e1NewB = (currentState >> 3) & 1;
  uint8_t e1Idx = (e1OldA << 3) | (e1OldB << 2) | (e1NewA << 1) | e1NewB;
  encLeft -= QUAD_TABLE[e1Idx];

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

// ============== IMU ==============

void setupIMU() {
  Serial.println("Initializing BNO085...");

  if (!bnoInit()) {
    Serial.println("BNO085 init failed!");
    while (1) delay(10);
  }

  Serial.println("BNO085 OK");

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

  heading = -(yaw - headingOffset);

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

  // Left motor (A)
  if (pwmL < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  }

  // Right motor (B)
  if (pwmR < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  } else {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  }

  // Apply deadband
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

// ============== ADAPTIVE FEEDFORWARD ==============

void resetAdaptive() {
  kffLeft = INITIAL_KFF;
  kffRight = INITIAL_KFF;
  historyIndex = 0;
  historyCount = 0;
}

void updateAdaptive(float velLeft, float velRight, float pwmLeft, float pwmRight) {
  // Only update when moving at reasonable speed (avoid division issues)
  const float MIN_VEL = 20.0;  // mm/s

  if (abs(velLeft) > MIN_VEL && abs(pwmLeft) > MIN_PWM) {
    float ratioL = abs(pwmLeft) / abs(velLeft);
    ratioHistoryLeft[historyIndex] = ratioL;
  } else {
    ratioHistoryLeft[historyIndex] = kffLeft;  // Keep current estimate
  }

  if (abs(velRight) > MIN_VEL && abs(pwmRight) > MIN_PWM) {
    float ratioR = abs(pwmRight) / abs(velRight);
    ratioHistoryRight[historyIndex] = ratioR;
  } else {
    ratioHistoryRight[historyIndex] = kffRight;
  }

  historyIndex = (historyIndex + 1) % SAMPLES_IN_WINDOW;
  if (historyCount < SAMPLES_IN_WINDOW) historyCount++;

  // Compute smoothed average
  if (historyCount > 10) {  // Wait for some samples
    float sumL = 0, sumR = 0;
    for (int i = 0; i < historyCount; i++) {
      sumL += ratioHistoryLeft[i];
      sumR += ratioHistoryRight[i];
    }
    float avgL = sumL / historyCount;
    float avgR = sumR / historyCount;

    // Gradually adapt toward the smoothed average
    kffLeft += ADAPT_RATE * (avgL - kffLeft);
    kffRight += ADAPT_RATE * (avgR - kffRight);

    // Clamp to reasonable range
    kffLeft = constrain(kffLeft, 0.3, 2.0);
    kffRight = constrain(kffRight, 0.3, 2.0);
  }
}

// ============== MOVEMENT ==============

float getHeadingError() {
  float error = targetHeading - heading;
  // Wrap to -180 to 180
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

void driveForward(float distanceMm) {
  Serial.print("Forward ");
  Serial.print(distanceMm);
  Serial.println(" mm");

  noInterrupts();
  long startLeft = encLeft;
  long startRight = encRight;
  long lastLeft = startLeft;
  long lastRight = startRight;
  interrupts();

  unsigned long lastTime = millis();
  float currentTargetVel = 0;  // For ramping
  int settleCount = 0;

  while (true) {
    updateIMU();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    // Calculate distance traveled
    float distLeft = (left - startLeft) * MM_PER_COUNT;
    float distRight = (right - startRight) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0;
    float distRemaining = distanceMm - distTraveled;

    // Calculate actual velocities
    float velLeft = (left - lastLeft) * MM_PER_COUNT / dt;
    float velRight = (right - lastRight) * MM_PER_COUNT / dt;
    lastLeft = left;
    lastRight = right;

    // Check if done
    if (distRemaining <= 0) {
      stop();
      Serial.print("Done. kffL=");
      Serial.print(kffLeft, 3);
      Serial.print(" kffR=");
      Serial.println(kffRight, 3);
      return;
    }

    // Calculate target velocity (slow down near end)
    float maxVel = TARGET_VELOCITY;
    // Deceleration zone: slow down when close
    float decelDist = 100.0;  // Start slowing 100mm from target
    if (distRemaining < decelDist) {
      maxVel = TARGET_VELOCITY * (distRemaining / decelDist);
      maxVel = max(maxVel, 30.0);  // Minimum creep speed
    }

    // Ramp up/down target velocity
    if (currentTargetVel < maxVel) {
      currentTargetVel += VELOCITY_RAMP * dt;
      if (currentTargetVel > maxVel) currentTargetVel = maxVel;
    } else if (currentTargetVel > maxVel) {
      currentTargetVel -= VELOCITY_RAMP * dt;
      if (currentTargetVel < maxVel) currentTargetVel = maxVel;
    }

    // Heading correction (differential velocity)
    float headingError = getHeadingError();
    float velCorrection = headingError * HEADING_Kp;

    float targetVelLeft = currentTargetVel + velCorrection;
    float targetVelRight = currentTargetVel - velCorrection;

    // Ensure minimum velocity when moving (to overcome deadband)
    const float MIN_TARGET_VEL = 50.0;
    if (targetVelLeft > 0 && targetVelLeft < MIN_TARGET_VEL) targetVelLeft = MIN_TARGET_VEL;
    if (targetVelLeft < 0 && targetVelLeft > -MIN_TARGET_VEL) targetVelLeft = -MIN_TARGET_VEL;
    if (targetVelRight > 0 && targetVelRight < MIN_TARGET_VEL) targetVelRight = MIN_TARGET_VEL;
    if (targetVelRight < 0 && targetVelRight > -MIN_TARGET_VEL) targetVelRight = -MIN_TARGET_VEL;

    // Adaptive feedforward: PWM = kff * targetVelocity
    float pwmL = kffLeft * targetVelLeft;
    float pwmR = kffRight * targetVelRight;

    // Small correction based on velocity error
    float velErrorL = targetVelLeft - velLeft;
    float velErrorR = targetVelRight - velRight;
    pwmL += velErrorL * 0.25;  // Small proportional correction
    pwmR += velErrorR * 0.25;

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    drive((int)pwmL, (int)pwmR);

    // Debug output for visualizer
    Serial.print("hdg:");
    Serial.print(heading, 1);
    Serial.print(" tgtHdg:");
    Serial.print(targetHeading);
    Serial.print(" tgtVelL:");
    Serial.print(targetVelLeft, 1);
    Serial.print(" tgtVelR:");
    Serial.print(targetVelRight, 1);
    Serial.print(" velL:");
    Serial.print(velLeft, 1);
    Serial.print(" velR:");
    Serial.print(velRight, 1);
    Serial.print(" pwmL:");
    Serial.print((int)pwmL);
    Serial.print(" pwmR:");
    Serial.print((int)pwmR);
    Serial.print(" ffL:");
    Serial.print(kffLeft * targetVelLeft, 1);
    Serial.print(" ffR:");
    Serial.println(kffRight * targetVelRight, 1);

    // Update adaptive coefficients
    updateAdaptive(velLeft, velRight, pwmL, pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void driveBackward(float distanceMm) {
  Serial.print("Backward ");
  Serial.print(distanceMm);
  Serial.println(" mm");

  noInterrupts();
  long startLeft = encLeft;
  long startRight = encRight;
  long lastLeft = startLeft;
  long lastRight = startRight;
  interrupts();

  unsigned long lastTime = millis();
  float currentTargetVel = 0;
  int settleCount = 0;

  while (true) {
    updateIMU();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    // Calculate distance traveled (backward is negative encoder change)
    float distLeft = (startLeft - left) * MM_PER_COUNT;
    float distRight = (startRight - right) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0;
    float distRemaining = distanceMm - distTraveled;

    // Calculate actual velocities
    float velLeft = (left - lastLeft) * MM_PER_COUNT / dt;
    float velRight = (right - lastRight) * MM_PER_COUNT / dt;
    lastLeft = left;
    lastRight = right;

    // Check if done
    if (distRemaining <= 0) {
      stop();
      Serial.print("Done. kffL=");
      Serial.print(kffLeft, 3);
      Serial.print(" kffR=");
      Serial.println(kffRight, 3);
      return;
    }

    // Calculate target velocity (negative for backward)
    float maxVel = TARGET_VELOCITY;
    float decelDist = 100.0;
    if (distRemaining < decelDist) {
      maxVel = TARGET_VELOCITY * (distRemaining / decelDist);
      maxVel = max(maxVel, 30.0);
    }

    if (currentTargetVel < maxVel) {
      currentTargetVel += VELOCITY_RAMP * dt;
      if (currentTargetVel > maxVel) currentTargetVel = maxVel;
    } else if (currentTargetVel > maxVel) {
      currentTargetVel -= VELOCITY_RAMP * dt;
      if (currentTargetVel < maxVel) currentTargetVel = maxVel;
    }

    // Heading correction
    float headingError = getHeadingError();
    float velCorrection = headingError * HEADING_Kp;

    // Negative velocity for backward
    float targetVelLeft = -currentTargetVel + velCorrection;
    float targetVelRight = -currentTargetVel - velCorrection;

    // Ensure minimum velocity when moving (to overcome deadband)
    const float MIN_TARGET_VEL = 50.0;
    if (targetVelLeft > 0 && targetVelLeft < MIN_TARGET_VEL) targetVelLeft = MIN_TARGET_VEL;
    if (targetVelLeft < 0 && targetVelLeft > -MIN_TARGET_VEL) targetVelLeft = -MIN_TARGET_VEL;
    if (targetVelRight > 0 && targetVelRight < MIN_TARGET_VEL) targetVelRight = MIN_TARGET_VEL;
    if (targetVelRight < 0 && targetVelRight > -MIN_TARGET_VEL) targetVelRight = -MIN_TARGET_VEL;

    // Adaptive feedforward
    float pwmL = kffLeft * targetVelLeft;
    float pwmR = kffRight * targetVelRight;

    // Small correction
    float velErrorL = targetVelLeft - velLeft;
    float velErrorR = targetVelRight - velRight;
    pwmL += velErrorL * 0.25;
    pwmR += velErrorR * 0.25;

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    drive((int)pwmL, (int)pwmR);

    // Debug output for visualizer
    Serial.print("hdg:");
    Serial.print(heading, 1);
    Serial.print(" tgtHdg:");
    Serial.print(targetHeading);
    Serial.print(" tgtVelL:");
    Serial.print(targetVelLeft, 1);
    Serial.print(" tgtVelR:");
    Serial.print(targetVelRight, 1);
    Serial.print(" velL:");
    Serial.print(velLeft, 1);
    Serial.print(" velR:");
    Serial.print(velRight, 1);
    Serial.print(" pwmL:");
    Serial.print((int)pwmL);
    Serial.print(" pwmR:");
    Serial.print((int)pwmR);
    Serial.print(" ffL:");
    Serial.print(kffLeft * targetVelLeft, 1);
    Serial.print(" ffR:");
    Serial.println(kffRight * targetVelRight, 1);

    // Update adaptive (use absolute values for learning)
    updateAdaptive(velLeft, velRight, pwmL, pwmR);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void turnLeft() {
  Serial.println("Turn left 90");
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
        Serial.println("Done");
        return;
      }
    }

    // Target turn velocity proportional to error
    float targetTurnVel = error * TURN_Kp;
    targetTurnVel = constrain(targetTurnVel, -TURN_VELOCITY, TURN_VELOCITY);

    // Left wheel forward, right wheel backward for left turn
    float pwmL = kffLeft * targetTurnVel;
    float pwmR = kffRight * (-targetTurnVel);

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    drive((int)pwmL, (int)pwmR);

    // Debug output
    Serial.print("hdg:");
    Serial.print(heading, 1);
    Serial.print(" tgtHdg:");
    Serial.println(targetHeading);

    delay(SAMPLE_INTERVAL_MS);
  }
}

void turnRight() {
  Serial.println("Turn right 90");
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
        Serial.println("Done");
        return;
      }
    }

    float targetTurnVel = error * TURN_Kp;
    targetTurnVel = constrain(targetTurnVel, -TURN_VELOCITY, TURN_VELOCITY);

    float pwmL = kffLeft * targetTurnVel;
    float pwmR = kffRight * (-targetTurnVel);

    pwmL = constrain(pwmL, -MAX_PWM, MAX_PWM);
    pwmR = constrain(pwmR, -MAX_PWM, MAX_PWM);

    drive((int)pwmL, (int)pwmR);

    // Debug output
    Serial.print("hdg:");
    Serial.print(heading, 1);
    Serial.print(" tgtHdg:");
    Serial.println(targetHeading);

    delay(SAMPLE_INTERVAL_MS);
  }
}

// ============== SEQUENCE RUNNER ==============

void runSequence(const char* seq) {
  Serial.print("Running sequence: ");
  Serial.println(seq);

  // Reset heading and adaptive coefficients
  updateIMU();
  headingOffset = heading + headingOffset;
  heading = 0;
  targetHeading = 0;
  resetAdaptive();

  for (int i = 0; seq[i] != '\0'; i++) {
    char cmd = seq[i];

    switch (cmd) {
      case 'f':
      case 'F':
        driveForward(DRIVE_DISTANCE);
        break;

      case 'b':
      case 'B':
        driveBackward(DRIVE_DISTANCE);
        break;

      case 'l':
      case 'L':
        turnLeft();
        break;

      case 'r':
      case 'R':
        turnRight();
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }

    delay(200);  // Pause between commands
  }

  Serial.println("Sequence complete!");
  Serial.print("Final kffL=");
  Serial.print(kffLeft, 3);
  Serial.print(" kffR=");
  Serial.println(kffRight, 3);
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
  resetAdaptive();

  Serial.println("Simple Sequence + Adaptive Velocity (BNO085)");
  Serial.print("Sequence: ");
  Serial.println(SEQUENCE);
  Serial.print("Drive distance: ");
  Serial.print(DRIVE_DISTANCE);
  Serial.println(" mm");
  Serial.print("Initial kff: ");
  Serial.println(INITIAL_KFF);
  Serial.println("Press button 4 to run");
}

unsigned long lastIdleOutput = 0;

void loop() {
  int buttonVal = digitalRead(BUTTON_PIN);

  if (lastButtonVal == HIGH && buttonVal == LOW) {
    delay(50);  // Debounce
    runSequence(SEQUENCE);
  }

  lastButtonVal = buttonVal;

  // Periodic heading output when idle
  if (millis() - lastIdleOutput > 10) {
    updateIMU();
    Serial.print("hdg:");
    Serial.print(heading, 1);
    Serial.print(" tgtHdg:");
    Serial.println(targetHeading);
    lastIdleOutput = millis();
  }
}
