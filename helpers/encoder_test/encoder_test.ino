/*
  ENCODER TEST - Check if encoder is reading properly

  This will continuously print the encoder count.
  Manually rotate the motor shaft and watch if the count changes.

  Connections:
  - Encoder P0: pin 34 (DB-50 pin 2)
  - Encoder P1: pin 35 (DB-50 pin 5)
  - Encoder +5V: Arduino 5V
  - Encoder GND: Arduino GND
*/

const int BASE_ENCODER_P0 = 34;
const int BASE_ENCODER_P1 = 35;

volatile long encoderCount = 0;
volatile int lastEncoded = 0;

void updateEncoder() {
  int MSB = digitalRead(BASE_ENCODER_P0);
  int LSB = digitalRead(BASE_ENCODER_P1);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;
}

void setup() {
  Serial.begin(9600);

  pinMode(BASE_ENCODER_P0, INPUT_PULLUP);
  pinMode(BASE_ENCODER_P1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BASE_ENCODER_P0), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BASE_ENCODER_P1), updateEncoder, CHANGE);

  Serial.println("Encoder Test");
  Serial.println("Manually rotate the motor shaft and watch the count change");
  Serial.println("If count doesn't change, encoder is not connected properly");
  Serial.println();
}

void loop() {
  static long lastPrintCount = 0;
  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime > 200) {
    Serial.print("Encoder count: ");
    Serial.print(encoderCount);

    if (encoderCount != lastPrintCount) {
      Serial.print("  (CHANGING!)");
    } else {
      Serial.print("  (not moving)");
    }

    Serial.println();

    lastPrintCount = encoderCount;
    lastPrintTime = millis();
  }
}
