// Simple Sequence Control
// Drive sequences like "frfl" (forward, right, forward, left)
// Maintains heading at 90° increments using gyro
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

// Control gains
const float HEADING_Kp = 3.0;      // Heading correction during straights
const float TURN_Kp = 2.0;         // Turn speed control
const float DIST_Kp = 1.0;         // Distance P gain (PWM per mm of error)
const float DIST_Ki = 0.0;         // Distance I gain
const float DIST_Kd = 0.1;         // Distance D gain
const int MIN_PWM = 50;            // Minimum PWM (deadband)
const int MAX_PWM = 255;           // Maximum PWM

// Tolerances
const float HEADING_TOLERANCE = 2.0;   // degrees
const float DISTANCE_TOLERANCE = 5.0;  // mm

// The sequence to run (edit this!)
const char* SEQUENCE = "frfrfrfr";  // Square: forward, right, x4

// ============== STATE ==============

volatile long encLeft = 0;
volatile long encRight = 0;

float heading = 0;          // Current heading from gyro (degrees)
int targetHeading = 0;      // Target heading (0, 90, 180, 270)
float gyroBiasZ = 0;

unsigned long lastGyroUpdate = 0;

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
  interrupts();

  float integral = 0;
  float lastError = 0;
  int settleCount = 0;

  while (true) {
    updateGyro();

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    float distLeft = (left - startLeft) * MM_PER_COUNT;
    float distRight = (right - startRight) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0;

    float distError = distanceMm - distTraveled;

    // Check if done
    if (abs(distError) < DISTANCE_TOLERANCE) {
      settleCount++;
      if (settleCount > 10) {
        stop();
        Serial.println("Done");
        return;
      }
    } else {
      settleCount = 0;
    }

    // Distance PID
    integral += distError * 0.01;  // dt ~= 10ms
    float derivative = (distError - lastError) / 0.01;
    lastError = distError;

    int basePWM = (int)(DIST_Kp * distError + DIST_Ki * integral + DIST_Kd * derivative);
    basePWM = constrain(basePWM, -MAX_PWM, MAX_PWM);

    // Heading correction
    float headingError = getHeadingError();
    int correction = (int)(headingError * HEADING_Kp);

    int pwmL = basePWM + correction;
    int pwmR = basePWM - correction;

    drive(pwmL, pwmR);

    delay(10);
  }
}

void driveBackward(float distanceMm) {
  Serial.print("Backward ");
  Serial.print(distanceMm);
  Serial.println(" mm");

  noInterrupts();
  long startLeft = encLeft;
  long startRight = encRight;
  interrupts();

  float integral = 0;
  float lastError = 0;
  int settleCount = 0;

  while (true) {
    updateGyro();

    noInterrupts();
    long left = encLeft;
    long right = encRight;
    interrupts();

    // For backward, distance is negative of forward
    float distLeft = (startLeft - left) * MM_PER_COUNT;
    float distRight = (startRight - right) * MM_PER_COUNT;
    float distTraveled = (distLeft + distRight) / 2.0;

    float distError = distanceMm - distTraveled;

    // Check if done
    if (abs(distError) < DISTANCE_TOLERANCE) {
      settleCount++;
      if (settleCount > 10) {
        stop();
        Serial.println("Done");
        return;
      }
    } else {
      settleCount = 0;
    }

    // Distance PID (negative because we want negative PWM for backward)
    integral += distError * 0.01;  // dt ~= 10ms
    float derivative = (distError - lastError) / 0.01;
    lastError = distError;

    int basePWM = (int)(DIST_Kp * distError + DIST_Ki * integral + DIST_Kd * derivative);
    basePWM = constrain(basePWM, -MAX_PWM, MAX_PWM);
    // Negate for backward motion
    basePWM = -basePWM;

    // Heading correction
    float headingError = getHeadingError();
    int correction = (int)(headingError * HEADING_Kp);

    int pwmL = basePWM + correction;
    int pwmR = basePWM - correction;

    drive(pwmL, pwmR);

    delay(10);
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

    int pwm = (int)(error * TURN_Kp);
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    // Turn in place: left forward, right backward (or vice versa)
    drive(pwm, -pwm);

    delay(10);
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

    int pwm = (int)(error * TURN_Kp);
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    drive(pwm, -pwm);

    delay(10);
  }
}

// ============== SEQUENCE RUNNER ==============

void runSequence(const char* seq) {
  Serial.print("Running sequence: ");
  Serial.println(seq);

  // Reset heading
  heading = 0;
  targetHeading = 0;

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

  Serial.println("Simple Sequence Control");
  Serial.print("Sequence: ");
  Serial.println(SEQUENCE);
  Serial.print("Drive distance: ");
  Serial.print(DRIVE_DISTANCE);
  Serial.println(" mm");
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
