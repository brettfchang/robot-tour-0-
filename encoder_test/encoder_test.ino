// Encoder Test - Quadrature encoders on A0-A3
// Rotate wheel by hand and watch the count

// Encoder pins
// Left encoder:  A0 (channel A), A1 (channel B)
// Right encoder: A2 (channel A), A3 (channel B)

// Encoder counts (volatile for ISR) - can go negative for reverse
volatile long encLeft = 0;
volatile long encRight = 0;

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

  // Right encoder: A2 = bit 2, A3 = bit 3
  uint8_t rightOldA = (lastPinState >> 2) & 1;
  uint8_t rightOldB = (lastPinState >> 3) & 1;
  uint8_t rightNewA = (currentState >> 2) & 1;
  uint8_t rightNewB = (currentState >> 3) & 1;
  uint8_t rightIdx = (rightOldA << 3) | (rightOldB << 2) | (rightNewA << 1) | rightNewB;
  encRight += QUAD_TABLE[rightIdx];

  lastPinState = currentState;
}

void setupPinChangeInterrupts() {
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

void setup() {
  Serial.begin(115200);

  setupPinChangeInterrupts();

  Serial.println("=== Quadrature Encoder Test ===");
  Serial.println("Rotate each wheel by hand exactly ONE full revolution.");
  Serial.println("Count should go + for forward, - for backward.");
  Serial.println("The absolute count = your COUNTS_PER_REV value.");
  Serial.println("Send 'r' to reset counts.\n");
}

void loop() {
  // Reset on 'r' key
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      noInterrupts();
      encLeft = 0;
      encRight = 0;
      interrupts();
      Serial.println("--- Counts reset ---\n");
    }
  }

  // Print counts
  noInterrupts();
  long left = encLeft;
  long right = encRight;
  interrupts();

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print("\t\tRight: ");
  Serial.println(right);

  delay(200);
}
