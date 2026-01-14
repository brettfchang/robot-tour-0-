// Motor & Encoder Mapping Test
// Figures out which encoder pins go with which motor and direction
//
// Instructions:
// 1. Upload and open Serial Monitor at 115200
// 2. Follow the prompts to test each motor and encoder
// 3. Write down the results!

// Motor pins (from your setup)
// Motor A pins
const int PWMA_PIN = 11;
const int AIN1_PIN = 10;
const int AIN2_PIN = 12;
// Motor B pins
const int PWMB_PIN = 5;
const int BIN1_PIN = 7;
const int BIN2_PIN = 6;
const int STBY_PIN = 9;

// Encoder pins
// "Enc0" = A0, A1
// "Enc1" = A2, A3

volatile long enc0 = 0;  // A0, A1
volatile long enc1 = 0;  // A2, A3
volatile uint8_t lastPinState = 0;

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

ISR(PCINT1_vect) {
  uint8_t currentState = PINC;

  // Enc0: A0 = bit 0, A1 = bit 1
  uint8_t e0OldA = (lastPinState >> 0) & 1;
  uint8_t e0OldB = (lastPinState >> 1) & 1;
  uint8_t e0NewA = (currentState >> 0) & 1;
  uint8_t e0NewB = (currentState >> 1) & 1;
  uint8_t e0Idx = (e0OldA << 3) | (e0OldB << 2) | (e0NewA << 1) | e0NewB;
  enc0 += QUAD_TABLE[e0Idx];

  // Enc1: A2 = bit 2, A3 = bit 3
  uint8_t e1OldA = (lastPinState >> 2) & 1;
  uint8_t e1OldB = (lastPinState >> 3) & 1;
  uint8_t e1NewA = (currentState >> 2) & 1;
  uint8_t e1NewB = (currentState >> 3) & 1;
  uint8_t e1Idx = (e1OldA << 3) | (e1OldB << 2) | (e1NewA << 1) | e1NewB;
  enc1 += QUAD_TABLE[e1Idx];

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

void stopMotors() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
}

void runMotorA(int pwm) {
  if (pwm >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }
  analogWrite(PWMA_PIN, abs(pwm));
}

void runMotorB(int pwm) {
  if (pwm >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }
  analogWrite(PWMB_PIN, abs(pwm));
}

void resetEncoders() {
  noInterrupts();
  enc0 = 0;
  enc1 = 0;
  interrupts();
}

void printEncoders() {
  noInterrupts();
  long e0 = enc0;
  long e1 = enc1;
  interrupts();
  Serial.print("Enc0 (A0,A1): ");
  Serial.print(e0);
  Serial.print("\t\tEnc1 (A2,A3): ");
  Serial.println(e1);
}

void waitForKey() {
  Serial.println("\nPress any key to continue...");
  while (!Serial.available()) {
    delay(10);
  }
  while (Serial.available()) Serial.read();  // Clear buffer
}

// Which motor pins are left vs right? (we'll figure this out)
bool motorAIsLeft = true;  // Initial guess

void runLeftMotor(int pwm) {
  if (motorAIsLeft) runMotorA(pwm);
  else runMotorB(pwm);
}

void runRightMotor(int pwm) {
  if (motorAIsLeft) runMotorB(pwm);
  else runMotorA(pwm);
}

void setup() {
  Serial.begin(115200);
  setupEncoders();
  setupMotors();
  stopMotors();

  delay(1000);

  Serial.println("========================================");
  Serial.println("   MOTOR & ENCODER MAPPING TEST");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Hold robot so wheels can spin freely.");
  Serial.println("Look at robot from BEHIND (like you're driving it).");

  waitForKey();
}

int testPhase = 0;

void loop() {
  switch (testPhase) {
    case 0:
      Serial.println("\n========== STEP 1: FIND LEFT MOTOR ==========");
      Serial.println("I'll spin Motor A. Tell me if it's the LEFT wheel.");
      Serial.println("(Left = your left when looking from behind robot)");
      waitForKey();

      resetEncoders();
      runMotorA(255);
      for (int i = 0; i < 20; i++) {
        delay(255);
        printEncoders();
      }
      stopMotors();

      Serial.println("\n>>> Was that the LEFT wheel? (y/n)");
      testPhase++;
      break;

    case 1:
      if (Serial.available()) {
        char c = Serial.read();
        while (Serial.available()) Serial.read();

        if (c == 'y' || c == 'Y') {
          motorAIsLeft = true;
          Serial.println("OK: Motor A = LEFT, Motor B = RIGHT");
        } else {
          motorAIsLeft = false;
          Serial.println("OK: Motor A = RIGHT, Motor B = LEFT");
        }
        testPhase++;
      }
      break;

    case 2:
      Serial.println("\n========== STEP 2: LEFT MOTOR FORWARD ==========");
      Serial.println("Running LEFT motor with +PWM...");
      Serial.println("Does the wheel spin FORWARD? (away from you)");
      waitForKey();

      resetEncoders();
      runLeftMotor(255);
      for (int i = 0; i < 20; i++) {
        delay(255);
        printEncoders();
      }
      stopMotors();

      Serial.println("\n>>> Which encoder changed? Enc0 or Enc1?");
      Serial.println(">>> Did it go + or -?");
      Serial.println(">>> Did the wheel go FORWARD or BACKWARD?");
      testPhase++;
      break;

    case 3:
      Serial.println("\n========== STEP 3: RIGHT MOTOR FORWARD ==========");
      Serial.println("Running RIGHT motor with +PWM...");
      waitForKey();

      resetEncoders();
      runRightMotor(255);
      for (int i = 0; i < 20; i++) {
        delay(255);
        printEncoders();
      }
      stopMotors();

      Serial.println("\n>>> Which encoder changed? Enc0 or Enc1?");
      Serial.println(">>> Did it go + or -?");
      Serial.println(">>> Did the wheel go FORWARD or BACKWARD?");
      testPhase++;
      break;

    case 4:
      Serial.println("\n========== STEP 4: BOTH FORWARD ==========");
      Serial.println("Running BOTH motors with +PWM...");
      Serial.println("Robot should go forward (or both wheels same direction)");
      waitForKey();

      resetEncoders();
      runLeftMotor(255);
      runRightMotor(255);
      for (int i = 0; i < 20; i++) {
        delay(255);
        printEncoders();
      }
      stopMotors();

      Serial.println("\n>>> Did both wheels go FORWARD?");
      Serial.println(">>> Or did they go opposite directions?");
      testPhase++;
      break;

    case 5:
      Serial.println("\n========================================");
      Serial.println("   MANUAL TEST - SPIN BY HAND");
      Serial.println("========================================");
      Serial.println();
      Serial.println("Motors off. Spin each wheel FORWARD by hand.");
      Serial.println();
      Serial.println("1. Spin LEFT wheel forward - note which encoder changes & sign");
      Serial.println("2. Spin RIGHT wheel forward - note which encoder changes & sign");
      Serial.println();
      Serial.println("Send 'r' to reset counters");
      Serial.println();
      resetEncoders();
      testPhase++;
      break;

    case 6:
      // Manual mode
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') {
          resetEncoders();
          Serial.println("--- Reset ---");
        }
      }
      printEncoders();
      delay(200);
      break;
  }
}
