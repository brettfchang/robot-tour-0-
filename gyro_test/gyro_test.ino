// Gyro Test - Just displays MPU6050 heading
// Rotate robot by hand and watch the heading change

#include <Wire.h>

const int MPU6050_ADDR = 0x68;

float heading = 0;        // radians
float gyroBiasZ = 0;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  // Set gyro range to ±250°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission();

  delay(100);

  // Calibrate
  Serial.println("=== Gyro Test ===");
  Serial.println("Calibrating... keep robot still!");

  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  gyroBiasZ = (float)sum / samples;

  Serial.print("Gyro bias: ");
  Serial.println(gyroBiasZ);
  Serial.println("\nRotate robot by hand. Send 'r' to reset heading.\n");

  lastUpdate = micros();
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void loop() {
  // Reset on 'r'
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      heading = 0;
      Serial.println("--- Heading reset ---\n");
    }
  }

  // Update heading
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  int16_t raw = readGyroZRaw();
  float gyroZ = -(raw - gyroBiasZ) / 131.0;  // °/s (inverted)
  heading += gyroZ * dt;

  // Print every 100ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.print(" deg  (raw: ");
    Serial.print(raw);
    Serial.print(", gyroZ: ");
    Serial.print(gyroZ);
    Serial.println(" deg/s)");
  }
}
