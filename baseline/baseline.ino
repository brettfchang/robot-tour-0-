// Baseline: Simple encoder odometry with position PID
// No cascaded loops, no adaptive velocity - just the essentials

// ============== CONFIGURATION ==============

// Motor pins (same as original)
const int PWMA_PIN = 11;
const int AIN1_PIN = 10;
const int AIN2_PIN = 12;
const int PWMB_PIN = 5;
const int BIN1_PIN = 7;
const int BIN2_PIN = 6;
const int STBY_PIN = 9;

// Encoder pins (quadrature on A0-A3)
// Left encoder:  A0 (channel A), A1 (channel B)
// Right encoder: A2 (channel A), A3 (channel B)

// Button pin
const int BUTTON_PIN = 4;

// Robot measurements - FILL THESE IN
const float WHEEL_DIAMETER_MM = 73.025;  // 2 7/8 inches
const float WHEEL_BASE_MM = 180.0;       // Distance between wheel centers
const int COUNTS_PER_REV = 900;          // Quadrature encoder counts per revolution

// Derived constants
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// Timing
const unsigned long LOOP_PERIOD_US = 20000;  // 50Hz

// PID gains - START WITH THESE, TUNE AS NEEDED
// Position PID (for straight driving)
float Kp_pos = 10.0;
float Ki_pos = 0.0;
float Kd_pos = 0.5;

// Heading PID (for keeping straight and turning)
float Kp_heading = 150.0;
float Ki_heading = 0.0;
float Kd_heading = 5.0;

// Motion limits
const int MAX_PWM = 255;          // Max speed
const int MIN_PWM = 80;           // Motor deadband
const float POS_TOLERANCE = 5.0;  // mm
const float HEADING_TOLERANCE = 0.035;  // ~2 degrees in radians
const int SETTLE_COUNT = 5;       // Consecutive readings in tolerance
const unsigned long TIMEOUT_MS = 5000;

// ============== GLOBAL STATE ==============

// Encoder counts (volatile for ISR)
volatile long encLeft = 0;
volatile long encRight = 0;

// Previous encoder values for delta calculation
long encLeftPrev = 0;
long encRightPrev = 0;

// Odometry state
float posX = 0;       // mm
float posY = 0;       // mm
float heading = 0;    // radians

// Timing
unsigned long lastUpdateUs = 0;

// Button state
int lastButtonVal = HIGH;

// PID state structure (must be defined before use)
struct PIDState {
  float integral;
  float lastError;
  float lastMeasurement;
  float derivFiltered;
};

PIDState pidPos = {0, 0, 0, 0};
PIDState pidHeading = {0, 0, 0, 0};

// ============== QUADRATURE ENCODER (Pin Change Interrupts) ==============

// Previous pin states for detecting changes
volatile uint8_t lastPinState = 0;

// Quadrature decoding lookup table
// Index: (oldA << 3) | (oldB << 2) | (newA << 1) | newB
// Value: -1, 0, or +1
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Pin Change Interrupt for Port C (A0-A5)
ISR(PCINT1_vect) {
  uint8_t currentState = PINC;

  // Left encoder: A0 = bit 0, A1 = bit 1
  uint8_t leftOldA = (lastPinState >> 0) & 1;
  uint8_t leftOldB = (lastPinState >> 1) & 1;
  uint8_t leftNewA = (currentState >> 0) & 1;
  uint8_t leftNewB = (currentState >> 1) & 1;
  uint8_t leftIdx = (leftOldA << 3) | (leftOldB << 2) | (leftNewA << 1) | leftNewB;
  encLeft += QUAD_TABLE[leftIdx];

  // Right encoder: A2 = bit 2, A3 = bit 3 (inverted)
  uint8_t rightOldA = (lastPinState >> 2) & 1;
  uint8_t rightOldB = (lastPinState >> 3) & 1;
  uint8_t rightNewA = (currentState >> 2) & 1;
  uint8_t rightNewB = (currentState >> 3) & 1;
  uint8_t rightIdx = (rightOldA << 3) | (rightOldB << 2) | (rightNewA << 1) | rightNewB;
  encRight -= QUAD_TABLE[rightIdx];  // Inverted

  lastPinState = currentState;
}

void setupEncoders() {
  // Set pins as inputs with pullup
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  // Store initial state
  lastPinState = PINC;

  // Enable Pin Change Interrupt for Port C (PCIE1)
  PCICR |= (1 << PCIE1);

  // Enable interrupts on A0-A3 (PCINT8-PCINT11)
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
}

// Read encoder values atomically (disable interrupts briefly)
void readEncoders(long &left, long &right) {
  noInterrupts();
  left = encLeft;
  right = encRight;
  interrupts();
}

// ============== ODOMETRY ==============

void updateOdometry(float dt) {
  long left, right;
  readEncoders(left, right);

  // Calculate deltas
  long deltaLeft = left - encLeftPrev;
  long deltaRight = right - encRightPrev;
  encLeftPrev = left;
  encRightPrev = right;

  // Convert to mm
  float distLeft = deltaLeft * MM_PER_COUNT;
  float distRight = deltaRight * MM_PER_COUNT;

  // Calculate movement
  float distCenter = (distLeft + distRight) / 2.0;
  float deltaHeading = (distRight - distLeft) / WHEEL_BASE_MM;

  // Update position using midpoint heading for better accuracy
  float midHeading = heading + deltaHeading / 2.0;
  posX += distCenter * cos(midHeading);
  posY += distCenter * sin(midHeading);
  heading += deltaHeading;

  // Normalize heading to [-PI, PI]
  while (heading > PI) heading -= 2 * PI;
  while (heading < -PI) heading += 2 * PI;
}

// Reset odometry to origin
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
}

// ============== PID CONTROLLER ==============

void resetPID(PIDState &pid) {
  pid.integral = 0;
  pid.lastError = 0;
  pid.lastMeasurement = 0;
  pid.derivFiltered = 0;
}

float computePID(float error, float measurement, float Kp, float Ki, float Kd,
                 PIDState &state, float dt, float maxIntegral) {
  // Proportional
  float P = Kp * error;

  // Integral with anti-windup
  state.integral += Ki * error * dt;
  state.integral = constrain(state.integral, -maxIntegral, maxIntegral);
  float I = state.integral;

  // Derivative on error (filtered)
  float derivative = (error - state.lastError) / dt;
  // Simple low-pass filter (alpha = 0.3 for more smoothing)
  state.derivFiltered = 0.3 * derivative + 0.7 * state.derivFiltered;
  float D = -Kd * state.derivFiltered;  // Negative for damping

  state.lastError = error;
  state.lastMeasurement = measurement;

  return P + I + D;
}

// ============== MOTOR CONTROL ==============

void enableMotors(bool en) {
  digitalWrite(STBY_PIN, en ? HIGH : LOW);
}

void drive(int powerL, int powerR) {
  powerL = constrain(powerL, -255, 255);
  powerR = constrain(powerR, -255, 255);

  // Apply deadband compensation
  if (powerL > 0) powerL = map(powerL, 0, 255, MIN_PWM, 255);
  else if (powerL < 0) powerL = map(powerL, -255, 0, -255, -MIN_PWM);

  if (powerR > 0) powerR = map(powerR, 0, 255, MIN_PWM, 255);
  else if (powerR < 0) powerR = map(powerR, -255, 0, -255, -MIN_PWM);

  // Left motor
  if (powerL < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  }

  // Right motor
  if (powerR < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  } else {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  }

  analogWrite(PWMA_PIN, abs(powerL));
  analogWrite(PWMB_PIN, abs(powerR));
}

void brake() {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
}

// ============== MOVEMENT FUNCTIONS ==============

// Drive straight for a given distance (mm)
// Positive = forward, negative = backward
bool driveStraight(float distanceMm) {
  // Calculate target position
  float targetX = posX + distanceMm * cos(heading);
  float targetY = posY + distanceMm * sin(heading);
  bool goingBackward = (distanceMm < 0);

  resetPID(pidPos);
  resetPID(pidHeading);

  int settleCount = 0;
  unsigned long startTime = millis();
  unsigned long lastUpdate = micros();
  unsigned long lastLog = 0;

  Serial.print("Driving to: ");
  Serial.print(targetX);
  Serial.print(", ");
  Serial.print(targetY);
  Serial.println(goingBackward ? " (backward)" : " (forward)");

  while (true) {
    unsigned long now = micros();
    if (now - lastUpdate >= LOOP_PERIOD_US) {
      float dt = (now - lastUpdate) / 1000000.0;
      lastUpdate += LOOP_PERIOD_US;

      // Update odometry
      updateOdometry(dt);

      // Vector to target
      float dx = targetX - posX;
      float dy = targetY - posY;
      float distError = sqrt(dx * dx + dy * dy);

      // Angle to target
      float angleToTarget = atan2(dy, dx);

      // Heading error - steer toward target
      float headingError;
      if (goingBackward) {
        // When going backward, we want our BACK to face the target
        // So desired heading is angleToTarget + 180°
        headingError = angleToTarget + PI - heading;
      } else {
        headingError = angleToTarget - heading;
      }
      // Normalize to [-PI, PI]
      while (headingError > PI) headingError -= 2 * PI;
      while (headingError < -PI) headingError += 2 * PI;

      // Compute PID outputs
      float baseSpeed = computePID(distError, 0, Kp_pos, Ki_pos, Kd_pos,
                                   pidPos, dt, MAX_PWM);
      float turnCorrection = computePID(headingError, 0, Kp_heading, Ki_heading, Kd_heading,
                                        pidHeading, dt, MAX_PWM / 2);

      // Limit base speed
      baseSpeed = constrain(baseSpeed, -MAX_PWM, MAX_PWM);

      // If going backward, negate base speed
      if (goingBackward) {
        baseSpeed = -baseSpeed;
      }

      // Calculate motor powers (constrain to valid PWM range)
      int powerL = constrain((int)(baseSpeed + turnCorrection), -255, 255);
      int powerR = constrain((int)(baseSpeed - turnCorrection), -255, 255);

      drive(powerL, powerR);

      // Log every 100ms
      unsigned long nowMs = millis();
      if (nowMs - lastLog >= 100) {
        lastLog = nowMs;
        Serial.print("dist:");
        Serial.print(distError);
        Serial.print(" hdgErr:");
        Serial.print(headingError * 180.0 / PI);
        Serial.print(" pwmL:");
        Serial.print(powerL);
        Serial.print(" pwmR:");
        Serial.println(powerR);
        printOdometry();
      }

      // Check if settled (close to target)
      if (distError < POS_TOLERANCE) {
        settleCount++;
        if (settleCount >= SETTLE_COUNT) {
          brake();
          Serial.println("Done - settled");
          return true;
        }
      } else {
        settleCount = 0;
      }

      // Timeout check
      if (millis() - startTime > TIMEOUT_MS) {
        brake();
        Serial.println("Done - timeout");
        return false;
      }
    }
  }
}

// Turn in place by a given angle (degrees)
// Positive = counter-clockwise, negative = clockwise
bool turn(float degrees) {
  float radians = degrees * PI / 180.0;
  float targetHeading = heading + radians;

  // Normalize target
  while (targetHeading > PI) targetHeading -= 2 * PI;
  while (targetHeading < -PI) targetHeading += 2 * PI;

  resetPID(pidHeading);

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

      // Update odometry
      updateOdometry(dt);

      // Calculate heading error
      float headingError = targetHeading - heading;
      // Normalize to [-PI, PI] for shortest path
      while (headingError > PI) headingError -= 2 * PI;
      while (headingError < -PI) headingError += 2 * PI;

      // Compute PID output (use 0 for measurement to avoid wrap-around issues)
      float turnSpeed = computePID(headingError, 0, Kp_heading, Ki_heading, Kd_heading,
                                   pidHeading, dt, MAX_PWM);

      // Limit turn speed
      turnSpeed = constrain(turnSpeed, -MAX_PWM, MAX_PWM);

      // Tank turn: opposite motor directions
      int powerL = constrain((int)(turnSpeed), -255, 255);
      int powerR = constrain((int)(-turnSpeed), -255, 255);

      drive(powerL, powerR);

      // Log every 100ms
      unsigned long nowMs = millis();
      if (nowMs - lastLog >= 100) {
        lastLog = nowMs;
        Serial.print("err:");
        Serial.print(headingError * 180.0 / PI);
        Serial.print("deg pwmL:");
        Serial.print(powerL);
        Serial.print(" pwmR:");
        Serial.println(powerR);
        printOdometry();
      }

      // Check if settled
      if (abs(headingError) < HEADING_TOLERANCE) {
        settleCount++;
        if (settleCount >= SETTLE_COUNT) {
          brake();
          Serial.println("Done - settled");
          return true;
        }
      } else {
        settleCount = 0;
      }

      // Timeout check
      if (millis() - startTime > TIMEOUT_MS) {
        brake();
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
  Serial.print(" Heading: ");
  Serial.print(heading * 180.0 / PI);
  Serial.println(" deg");
}

// ============== TEST SEQUENCE ==============

void runTestSequence() {
  Serial.println("\n=== Starting Test Sequence ===");
  resetOdometry();
  enableMotors(true);

  // Drive a square: 300mm sides
  for (int i = 0; i < 4; i++) {
    Serial.print("\n--- Side ");
    Serial.print(i + 1);
    Serial.println(" ---");

    driveStraight(300);
    printOdometry();
    delay(500);

    turn(90);
    printOdometry();
    delay(500);
  }

  enableMotors(false);

  Serial.println("\n=== Test Complete ===");
  Serial.println("Final position (should be near 0,0):");
  printOdometry();
}

// ============== SETUP & LOOP ==============

void setup() {
  Serial.begin(115200);  // Faster baud rate for less timing impact

  // Motor pins
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  // Quadrature encoders on A0-A3
  setupEncoders();

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  enableMotors(false);

  Serial.println("Baseline Robot Control");
  Serial.println("Press button to run test sequence");
  Serial.println("Commands via Serial:");
  Serial.println("  f<dist>  - drive forward (mm), e.g. f500");
  Serial.println("  b<dist>  - drive backward (mm), e.g. b200");
  Serial.println("  l<deg>   - turn left (deg), e.g. l90");
  Serial.println("  r<deg>   - turn right (deg), e.g. r90");
  Serial.println("  p        - print odometry");
  Serial.println("  z        - zero/reset odometry");
}

void loop() {
  // Button check
  int buttonVal = digitalRead(BUTTON_PIN);
  if (lastButtonVal == HIGH && buttonVal == LOW) {
    delay(50);  // Debounce
    runTestSequence();
  }
  lastButtonVal = buttonVal;

  // Serial command handling
  if (Serial.available()) {
    char cmd = Serial.read();
    float value = Serial.parseFloat();

    enableMotors(true);

    switch (cmd) {
      case 'f':
        driveStraight(value);
        break;
      case 'b':
        driveStraight(-value);
        break;
      case 'l':
        turn(value);
        break;
      case 'r':
        turn(-value);
        break;
      case 'p':
        printOdometry();
        break;
      case 'z':
        resetOdometry();
        Serial.println("Odometry reset");
        break;
    }

    enableMotors(false);
    printOdometry();

    // Clear any remaining serial data
    while (Serial.available()) Serial.read();
  }
}
