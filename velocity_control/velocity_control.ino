// Velocity Control: Closed-loop velocity PID for each wheel
// Button 4 runs a 500mm square (right turns)
// Uses MPU6050 gyro for heading

#include <Wire.h>

// ============== CONFIGURATION ==============

// MPU6050 I2C address
const int MPU6050_ADDR = 0x68;

// Motor pins
const int PWMA_PIN = 11;
const int AIN1_PIN = 10;
const int AIN2_PIN = 12;
const int PWMB_PIN = 5;
const int BIN1_PIN = 7;
const int BIN2_PIN = 6;
const int STBY_PIN = 9;

// Button pin
const int BUTTON_PIN = 4;

// Robot measurements
const float WHEEL_DIAMETER_MM = 73.025;  // 2 7/8 inches
const float WHEEL_BASE_MM = 180.0;
const int COUNTS_PER_REV = 900;

// Derived constants
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// Timing
const unsigned long LOOP_PERIOD_US = 20000;  // 50Hz

// Velocity PID gains (inner loop - controls PWM to achieve target velocity)
float Kp_vel = 4.0;
float Ki_vel = 0.4;
float Kd_vel = 0.0;
const float MAX_VELOCITY = 300.0;  // mm/s max speed

// Position PID gains (outer loop - outputs target velocity)
float Kp_pos = 2.0;
float Ki_pos = 0.0;
float Kd_pos = 0.1;

// Heading PID gains
float Kp_heading = 80.0;
float Ki_heading = 0.0;
float Kd_heading = 5.0;
float Kd_heading_turn = 20.0;

float Kp_heading_straight = 500.0;

// Motion limits
const int MAX_PWM = 255;
const int MIN_PWM = 50;
const float POS_TOLERANCE = 5.0;
const float HEADING_TOLERANCE = 0.035;
const int SETTLE_COUNT = 5;
const unsigned long TIMEOUT_MS = 5000;

// ============== GLOBAL STATE ==============

volatile long encLeft = 0;
volatile long encRight = 0;

long encLeftPrev = 0;
long encRightPrev = 0;

// Wheel velocities (mm/s)
float velLeft = 0;
float velRight = 0;

// Measured odometry
float posX = 0;
float posY = 0;
float heading = 0;
float headingEncoder = 0;

// Gyro calibration
float gyroBiasZ = 0;

// Expected position (ideal path)
float expectedX = 0;
float expectedY = 0;
float expectedHeading = 0;

int lastButtonVal = HIGH;

struct PIDState {
  float integral;
  float lastError;
  float lastMeasurement;
  float derivFiltered;
};

PIDState pidPos = {0, 0, 0, 0};
PIDState pidHeading = {0, 0, 0, 0};
PIDState pidVelLeft = {0, 0, 0, 0};
PIDState pidVelRight = {0, 0, 0, 0};

// ============== QUADRATURE ENCODER ==============

volatile uint8_t lastPinState = 0;

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

ISR(PCINT1_vect) {
  uint8_t currentState = PINC;

  uint8_t leftOldA = (lastPinState >> 0) & 1;
  uint8_t leftOldB = (lastPinState >> 1) & 1;
  uint8_t leftNewA = (currentState >> 0) & 1;
  uint8_t leftNewB = (currentState >> 1) & 1;
  uint8_t leftIdx = (leftOldA << 3) | (leftOldB << 2) | (leftNewA << 1) | leftNewB;
  encLeft += QUAD_TABLE[leftIdx];

  uint8_t rightOldA = (lastPinState >> 2) & 1;
  uint8_t rightOldB = (lastPinState >> 3) & 1;
  uint8_t rightNewA = (currentState >> 2) & 1;
  uint8_t rightNewB = (currentState >> 3) & 1;
  uint8_t rightIdx = (rightOldA << 3) | (rightOldB << 2) | (rightNewA << 1) | rightNewB;
  encRight -= QUAD_TABLE[rightIdx];  // Inverted

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

void readEncoders(long &left, long &right) {
  noInterrupts();
  left = encLeft;
  right = encRight;
  interrupts();
}

// ============== MPU6050 GYRO ==============

void setupMPU6050() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  delay(100);

  Serial.println("Calibrating gyro... keep robot still!");
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  gyroBiasZ = (float)sum / samples;
  Serial.print("Gyro bias: ");
  Serial.println(gyroBiasZ);
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  return Wire.read() << 8 | Wire.read();
}

float readGyroZ() {
  int16_t raw = readGyroZRaw();
  float gyroZ = -(raw - gyroBiasZ) / 131.0;
  return gyroZ * PI / 180.0;
}

// ============== ODOMETRY ==============

void updateOdometry(float dt) {
  long left, right;
  readEncoders(left, right);

  long deltaLeft = left - encLeftPrev;
  long deltaRight = right - encRightPrev;
  encLeftPrev = left;
  encRightPrev = right;

  float distLeft = deltaLeft * MM_PER_COUNT;
  float distRight = deltaRight * MM_PER_COUNT;

  // Calculate wheel velocities (mm/s)
  velLeft = distLeft / dt;
  velRight = distRight / dt;

  float distCenter = (distLeft + distRight) / 2.0;

  float deltaHeadingEnc = (distRight - distLeft) / WHEEL_BASE_MM;
  headingEncoder += deltaHeadingEnc;

  float gyroZ = readGyroZ();
  heading += gyroZ * dt;

  float midHeading = heading - (gyroZ * dt) / 2.0;
  posX += distCenter * cos(midHeading);
  posY += distCenter * sin(midHeading);

  while (heading > PI) heading -= 2 * PI;
  while (heading < -PI) heading += 2 * PI;
  while (headingEncoder > PI) headingEncoder -= 2 * PI;
  while (headingEncoder < -PI) headingEncoder += 2 * PI;
}

void resetOdometry() {
  noInterrupts();
  encLeft = 0;
  encRight = 0;
  interrupts();
  encLeftPrev = 0;
  encRightPrev = 0;
  posX = 0;
  posY = 0;
  heading = 0;
  headingEncoder = 0;
  velLeft = 0;
  velRight = 0;
  expectedX = 0;
  expectedY = 0;
  expectedHeading = 0;
}

// ============== PID ==============

void resetPID(PIDState &pid) {
  pid.integral = 0;
  pid.lastError = 0;
  pid.lastMeasurement = 0;
  pid.derivFiltered = 0;
}

float computePID(float error, float measurement, float Kp, float Ki, float Kd,
                 PIDState &state, float dt, float maxIntegral) {
  float P = Kp * error;

  state.integral += Ki * error * dt;
  state.integral = constrain(state.integral, -maxIntegral, maxIntegral);
  float I = state.integral;

  float derivative = (error - state.lastError) / dt;
  state.derivFiltered = 0.3 * derivative + 0.7 * state.derivFiltered;
  float D = -Kd * state.derivFiltered;

  state.lastError = error;
  state.lastMeasurement = measurement;

  return P + I + D;
}

// ============== MOTOR CONTROL ==============

void enableMotors(bool en) {
  digitalWrite(STBY_PIN, en ? HIGH : LOW);
}

// Low-level drive - applies PWM directly (no deadband compensation here)
void driveRaw(int pwmL, int pwmR) {
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

  analogWrite(PWMA_PIN, abs(pwmL));
  analogWrite(PWMB_PIN, abs(pwmR));
}

void brake() {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
}

// Closed-loop velocity control - takes target velocities (mm/s), outputs PWM
// Returns the PWM values applied
void driveVelocity(float targetVelL, float targetVelR, float dt, int &pwmLOut, int &pwmROut) {
  // Velocity errors
  float velErrorL = targetVelL - velLeft;
  float velErrorR = targetVelR - velRight;

  // Velocity PIDs output PWM adjustments
  float pwmL = computePID(velErrorL, velLeft, Kp_vel, Ki_vel, Kd_vel,
                          pidVelLeft, dt, MAX_PWM);
  float pwmR = computePID(velErrorR, velRight, Kp_vel, Ki_vel, Kd_vel,
                          pidVelRight, dt, MAX_PWM);

  // Add feedforward based on target velocity (rough estimate)
  // This helps the PID not have to do all the work
  float ffL = targetVelL * 0.5;  // Feedforward gain
  float ffR = targetVelR * 0.5;

  pwmL += ffL;
  pwmR += ffR;

  // Apply deadband compensation
  if (pwmL > 0) pwmL = max(pwmL, (float)MIN_PWM);
  else if (pwmL < 0) pwmL = min(pwmL, (float)-MIN_PWM);
  else if (abs(targetVelL) < 1.0) pwmL = 0;  // Allow zero if target is zero

  if (pwmR > 0) pwmR = max(pwmR, (float)MIN_PWM);
  else if (pwmR < 0) pwmR = min(pwmR, (float)-MIN_PWM);
  else if (abs(targetVelR) < 1.0) pwmR = 0;

  pwmLOut = constrain((int)pwmL, -255, 255);
  pwmROut = constrain((int)pwmR, -255, 255);

  driveRaw(pwmLOut, pwmROut);
}

// ============== MOVEMENT ==============

bool driveStraight(float distanceMm) {
  float targetX = expectedX + distanceMm * cos(expectedHeading);
  float targetY = expectedY + distanceMm * sin(expectedHeading);
  bool goingBackward = (distanceMm < 0);
  float totalDist = abs(distanceMm);

  float startX = posX;
  float startY = posY;

  resetPID(pidPos);
  resetPID(pidHeading);
  resetPID(pidVelLeft);
  resetPID(pidVelRight);

  int settleCount = 0;
  unsigned long startTime = millis();
  unsigned long lastUpdate = micros();
  unsigned long lastLog = 0;

  Serial.print("Driving to: ");
  Serial.print(targetX);
  Serial.print(", ");
  Serial.println(targetY);

  while (true) {
    unsigned long now = micros();
    if (now - lastUpdate >= LOOP_PERIOD_US) {
      float dt = (now - lastUpdate) / 1000000.0;
      lastUpdate += LOOP_PERIOD_US;

      updateOdometry(dt);

      float distTraveled = sqrt((posX - startX) * (posX - startX) + (posY - startY) * (posY - startY));
      float distError = totalDist - distTraveled;
      if (distError < 0) distError = 0;

      float dx = targetX - posX;
      float dy = targetY - posY;
      float angleToTarget = atan2(dy, dx);

      float headingError;
      if (goingBackward) {
        headingError = angleToTarget + PI - heading;
      } else {
        headingError = angleToTarget - heading;
      }
      while (headingError > PI) headingError -= 2 * PI;
      while (headingError < -PI) headingError += 2 * PI;

      float progress = distTraveled / totalDist;
      progress = constrain(progress, 0.0, 1.0);
      float headingCorrectionScale = 1.0 - progress;

      // Outer loop: position error -> target velocity
      float targetVel = computePID(distError, 0, Kp_pos, Ki_pos, Kd_pos,
                                   pidPos, dt, MAX_VELOCITY);
      targetVel = constrain(targetVel, -MAX_VELOCITY, MAX_VELOCITY);

      // Heading correction -> differential velocity
      float turnCorrection = computePID(headingError, 0, Kp_heading_straight, Ki_heading, Kd_heading,
                                        pidHeading, dt, MAX_VELOCITY / 2);
      turnCorrection *= headingCorrectionScale;

      if (goingBackward) {
        targetVel = -targetVel;
      }

      // Convert to wheel velocities
      float targetVelL = targetVel + turnCorrection;
      float targetVelR = targetVel - turnCorrection;

      // Inner loop: velocity control
      int pwmL, pwmR;
      driveVelocity(targetVelL, targetVelR, dt, pwmL, pwmR);

      unsigned long nowMs = millis();
      if (nowMs - lastLog >= 100) {
        lastLog = nowMs;
        Serial.print("dist:");
        Serial.print(distError);
        Serial.print(" tgtVel:");
        Serial.print(targetVel);
        Serial.print(" velL:");
        Serial.print(velLeft);
        Serial.print(" velR:");
        Serial.print(velRight);
        Serial.print(" pwmL:");
        Serial.print(pwmL);
        Serial.print(" pwmR:");
        Serial.println(pwmR);
        printOdometry();
      }

      if (distError < POS_TOLERANCE) {
        settleCount++;
        if (settleCount >= SETTLE_COUNT) {
          brake();
          expectedX = targetX;
          expectedY = targetY;
          Serial.println("Done - settled");
          return true;
        }
      } else {
        settleCount = 0;
      }

      if (millis() - startTime > TIMEOUT_MS) {
        brake();
        expectedX = targetX;
        expectedY = targetY;
        Serial.println("Done - timeout");
        return false;
      }
    }
  }
}

bool turn(float degrees) {
  float radians = degrees * PI / 180.0;
  float targetHeading = expectedHeading + radians;
  bool turningLeft = (degrees > 0);

  while (targetHeading > PI) targetHeading -= 2 * PI;
  while (targetHeading < -PI) targetHeading += 2 * PI;

  resetPID(pidHeading);
  resetPID(pidVelLeft);
  resetPID(pidVelRight);

  int settleCount = 0;
  unsigned long startTime = millis();
  unsigned long lastUpdate = micros();
  unsigned long lastLog = 0;

  Serial.print("Turning: ");
  Serial.print(degrees);
  Serial.println(" deg");

  while (true) {
    unsigned long now = micros();
    if (now - lastUpdate >= LOOP_PERIOD_US) {
      float dt = (now - lastUpdate) / 1000000.0;
      lastUpdate += LOOP_PERIOD_US;

      updateOdometry(dt);

      float headingError = targetHeading - heading;
      while (headingError > PI) headingError -= 2 * PI;
      while (headingError < -PI) headingError += 2 * PI;

      // Outer loop: heading error -> turn velocity
      float turnVel = computePID(headingError, 0, Kp_heading, Ki_heading, Kd_heading_turn,
                                 pidHeading, dt, MAX_VELOCITY);
      turnVel = constrain(turnVel, -MAX_VELOCITY, MAX_VELOCITY);

      // Prevent reverse turning
      bool wouldTurnWrongWay = (turningLeft && headingError < 0) || (!turningLeft && headingError > 0);
      if (wouldTurnWrongWay) {
        turnVel = 0;
      }

      // Stop zone
      if (abs(headingError) < 0.1) {
        turnVel = 0;
      }

      // Convert to wheel velocities (differential drive)
      float targetVelL = turnVel;
      float targetVelR = -turnVel;

      // Inner loop: velocity control
      int pwmL, pwmR;
      driveVelocity(targetVelL, targetVelR, dt, pwmL, pwmR);

      unsigned long nowMs = millis();
      if (nowMs - lastLog >= 100) {
        lastLog = nowMs;
        Serial.print("err:");
        Serial.print(headingError * 180.0 / PI);
        Serial.print("deg turnVel:");
        Serial.print(turnVel);
        Serial.print(" velL:");
        Serial.print(velLeft);
        Serial.print(" velR:");
        Serial.print(velRight);
        Serial.print(" pwmL:");
        Serial.print(pwmL);
        Serial.print(" pwmR:");
        Serial.println(pwmR);
        printOdometry();
      }

      if (abs(headingError) < HEADING_TOLERANCE) {
        settleCount++;
        if (settleCount >= SETTLE_COUNT) {
          brake();
          expectedHeading = targetHeading;
          Serial.println("Done - settled");
          return true;
        }
      } else {
        settleCount = 0;
      }

      if (millis() - startTime > TIMEOUT_MS) {
        brake();
        expectedHeading = targetHeading;
        Serial.println("Done - timeout");
        return false;
      }
    }
  }
}

// ============== DEBUG ==============

void printOdometry() {
  Serial.print("X: ");
  Serial.print(posX);
  Serial.print(" Y: ");
  Serial.print(posY);
  Serial.print(" Hdg(gyro): ");
  Serial.print(heading * 180.0 / PI);
  Serial.print(" (exp: ");
  Serial.print(expectedX);
  Serial.print(", ");
  Serial.print(expectedY);
  Serial.print(", ");
  Serial.print(expectedHeading * 180.0 / PI);
  Serial.println(")");
}

// ============== SQUARE SEQUENCE ==============

void runSquare() {
  Serial.println("\n=== Running 500mm Square (Right Turns) ===");
  resetOdometry();
  enableMotors(true);

  for (int i = 0; i < 4; i++) {
    Serial.print("\n--- Side ");
    Serial.print(i + 1);
    Serial.println(" ---");

    driveStraight(500);
    printOdometry();
    delay(300);

    turn(-90);
    printOdometry();
    delay(300);
  }

  enableMotors(false);

  Serial.println("\n=== Square Complete ===");
  Serial.println("Final position (should be near 0,0):");
  printOdometry();
}

// ============== SETUP & LOOP ==============

void setup() {
  Serial.begin(115200);

  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  setupEncoders();
  setupMPU6050();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  enableMotors(false);

  Serial.println("Velocity Control Test (Closed-loop velocity)");
  Serial.println("Press button 4 to run 500mm square (right turns)");
}

void loop() {
  int buttonVal = digitalRead(BUTTON_PIN);
  if (lastButtonVal == HIGH && buttonVal == LOW) {
    delay(50);
    runSquare();
  }
  lastButtonVal = buttonVal;
}
