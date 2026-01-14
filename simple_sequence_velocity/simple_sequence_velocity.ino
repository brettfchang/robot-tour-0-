// Simple Sequence Control with Adaptive Velocity
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

// MPU6050
const int MPU6050_ADDR = 0x68;

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
const float VELOCITY_RAMP = 50.0;      // mm/s^2 acceleration limit

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

float heading = 0;          // Current heading from gyro (degrees)
int targetHeading = 0;      // Target heading (0, 90, 180, 270)
float gyroBiasZ = 0;

unsigned long lastGyroUpdate = 0;

// Adaptive feedforward state
float kffLeft = INITIAL_KFF;   // Learned coefficient for left wheel
float kffRight = INITIAL_KFF;  // Learned coefficient for right wheel

// Circular buffers for smoothing (stores PWM/velocity ratios)
float ratioHistoryLeft[100];
float ratioHistoryRight[100];
int historyIndex = 0;
int historyCount = 0;

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

// ============== GYRO ==============

void setupGyro() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission();

  delay(100);

  // Calibrate
  Serial.println("Calibrating gyro...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroBiasZ = sum / 500.0;
  Serial.print("Gyro bias: ");
  Serial.println(gyroBiasZ);

  lastGyroUpdate = micros();
}

void updateGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  int16_t raw = Wire.read() << 8 | Wire.read();

  unsigned long now = micros();
  float dt = (now - lastGyroUpdate) / 1000000.0;
  lastGyroUpdate = now;

  float gyroZ = -(raw - gyroBiasZ) / 131.0;  // °/s, inverted
  heading += gyroZ * dt;
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
    updateGyro();

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
    if (abs(distRemaining) < DISTANCE_TOLERANCE) {
      settleCount++;
      if (settleCount > 10) {
        stop();
        Serial.print("Done. kffL=");
        Serial.print(kffLeft, 3);
        Serial.print(" kffR=");
        Serial.println(kffRight, 3);
        return;
      }
    } else {
      settleCount = 0;
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
    Serial.print("tgtVelL:");
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
    updateGyro();

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
    if (abs(distRemaining) < DISTANCE_TOLERANCE) {
      settleCount++;
      if (settleCount > 10) {
        stop();
        Serial.print("Done. kffL=");
        Serial.print(kffLeft, 3);
        Serial.print(" kffR=");
        Serial.println(kffRight, 3);
        return;
      }
    } else {
      settleCount = 0;
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
    Serial.print("tgtVelL:");
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
    updateGyro();

    float error = getHeadingError();

    if (abs(error) < HEADING_TOLERANCE) {
      stop();
      delay(100);
      updateGyro();
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

    delay(SAMPLE_INTERVAL_MS);
  }
}

void turnRight() {
  Serial.println("Turn right 90");
  targetHeading -= 90;
  if (targetHeading < 0) targetHeading += 360;

  while (true) {
    updateGyro();

    float error = getHeadingError();

    if (abs(error) < HEADING_TOLERANCE) {
      stop();
      delay(100);
      updateGyro();
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

    delay(SAMPLE_INTERVAL_MS);
  }
}

// ============== SEQUENCE RUNNER ==============

void runSequence(const char* seq) {
  Serial.print("Running sequence: ");
  Serial.println(seq);

  // Reset heading and adaptive coefficients
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
  setupGyro();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  stop();
  resetAdaptive();

  Serial.println("Simple Sequence + Adaptive Velocity");
  Serial.print("Sequence: ");
  Serial.println(SEQUENCE);
  Serial.print("Drive distance: ");
  Serial.print(DRIVE_DISTANCE);
  Serial.println(" mm");
  Serial.print("Initial kff: ");
  Serial.println(INITIAL_KFF);
  Serial.println("Press button 4 to run");
}

void loop() {
  int buttonVal = digitalRead(BUTTON_PIN);

  if (lastButtonVal == HIGH && buttonVal == LOW) {
    delay(50);  // Debounce
    runSequence(SEQUENCE);
  }

  lastButtonVal = buttonVal;
}
