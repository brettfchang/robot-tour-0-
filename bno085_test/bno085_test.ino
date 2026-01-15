// BNO085 Minimal Driver Test
// Outputs heading at 10Hz for testing
//
// Wiring:
//   VCC -> 5V (or 3.3V)
//   GND -> GND
//   SDA -> A4
//   SCL -> A5
//   RST -> Pin 8

#include <Wire.h>

const int BNO085_RST_PIN = 8;
const uint8_t BNO085_ADDR = 0x4A;

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
      } else {
        // Debug: show rejected values
        Serial.print(F("REJ: "));
        Serial.print(ti); Serial.print(F(" "));
        Serial.print(tj); Serial.print(F(" "));
        Serial.print(tk); Serial.print(F(" "));
        Serial.println(tr);
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

float getHeading() {
  float qi = quat[0];
  float qj = quat[1];
  float qk = quat[2];
  float qr = quat[3];

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  return atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("BNO085 Test"));
  Serial.println(F("Initializing..."));

  if (!bnoInit()) {
    Serial.println(F("BNO085 FAILED!"));
    while (1) {
      delay(1000);
    }
  }

  Serial.println(F("BNO085 OK!"));
  Serial.println(F("Rotate sensor to see heading change"));
  Serial.println();
}

void loop() {
  if (bnoUpdate()) {
    float h = getHeading();

    Serial.print(F("Hdg: "));
    if (h >= 0) Serial.print(' ');
    if (abs(h) < 100) Serial.print(' ');
    if (abs(h) < 10) Serial.print(' ');
    Serial.print(h, 1);
    Serial.print(F("  ["));

    int pos = map((int)h, -180, 180, 0, 20);
    for (int i = 0; i < 20; i++) {
      if (i == 10) Serial.print('|');
      else if (i == pos) Serial.print('*');
      else Serial.print('-');
    }
    Serial.println(F("]"));
  }

  delay(100);
}
