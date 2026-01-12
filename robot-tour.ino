// Simplified motor test - no encoders or gyro feedback

// Motor pins
const int pwma_pin = 11;
const int ain1_pin = 10;
const int ain2_pin = 12;
const int pwmb_pin = 5;
const int bin1_pin = 7;
const int bin2_pin = 6;
const int stby_pin = 9;

// Button pin (uses internal pull-up, connect button to GND)
const int button_pin = 4;

int last_button_val = HIGH;

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(pwma_pin, OUTPUT);
  pinMode(ain1_pin, OUTPUT);
  pinMode(ain2_pin, OUTPUT);
  pinMode(pwmb_pin, OUTPUT);
  pinMode(bin1_pin, OUTPUT);
  pinMode(bin2_pin, OUTPUT);
  pinMode(stby_pin, OUTPUT);

  // Button with internal pull-up
  pinMode(button_pin, INPUT_PULLUP);

  Serial.println("Ready. Press button to test motors.");
}

void loop() {
  int current_button_val = digitalRead(button_pin);

  // Button pressed (HIGH to LOW with pull-up)
  if (last_button_val == HIGH && current_button_val == LOW) {
    Serial.println("Running motor test...");
    runMotorTest();
  }

  last_button_val = current_button_val;
}

void runMotorTest() {
  enableMotors(true);

  Serial.println("Forward");
  drive(150, 150);
  delay(1000);
  brake();
  delay(500);

  Serial.println("Backward");
  drive(-150, -150);
  delay(1000);
  brake();
  delay(500);

  Serial.println("Turn left");
  drive(-150, 150);
  delay(500);
  brake();
  delay(500);

  Serial.println("Turn right");
  drive(150, -150);
  delay(500);
  brake();
  delay(500);

  Serial.println("Done!");
  enableMotors(false);
}

void enableMotors(bool en) {
  digitalWrite(stby_pin, en ? HIGH : LOW);
}

void drive(int power_l, int power_r) {
  power_l = constrain(power_l, -255, 255);
  power_r = constrain(power_r, -255, 255);

  // Left motor direction (flipped)
  if (power_l < 0) {
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
  } else {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
  }

  // Right motor direction (flipped)
  if (power_r < 0) {
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
  } else {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  }

  analogWrite(pwma_pin, abs(power_l));
  analogWrite(pwmb_pin, abs(power_r));
}

void brake() {
  digitalWrite(ain1_pin, HIGH);
  digitalWrite(ain2_pin, HIGH);
  digitalWrite(bin1_pin, HIGH);
  digitalWrite(bin2_pin, HIGH);
  analogWrite(pwma_pin, 0);
  analogWrite(pwmb_pin, 0);
}
