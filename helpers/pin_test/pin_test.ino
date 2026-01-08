/*
SCORBOT controller pin test for Arduino Mega

test wiring harness for pins 34-52 to DB-50 breakout board
with DB50 plug DISCONNECTED

TEST PROCEDURE:
1. Upload this sketch to Arduino Mega
2. Open Serial Monitor (9600 baud)
3. Phase 1 checks for dangerous shorts to 5V (automatic)
4. Phase 2 tests wiring: touch each DB-50 pin to GND
5. Arduino will report which pin detected the connection
6. Verify against your wiring table

NOTE: Some DB-50 pins have 47Ω resistors to 5V for encoder LEDs.
Phase 1 detects if pins are stuck HIGH (shorted to 5V).
*/

#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port
  }

  Serial.println("===================================");
  Serial.println("SCORBOT Arduino Mega Pin Test");
  Serial.println("===================================");
  Serial.println();

  // PHASE 1: SHORT TO 5V DETECTION
  Serial.println("PHASE 1: Testing for shorts to 5V...");
  Serial.println("(DB-50 should be DISCONNECTED)");
  Serial.println();

  boolean shortTo5V = false;

  for(int pin = 34; pin <= 52; pin++) {
    // Try to pull pin LOW
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(100);

    // Set back to input and check if it stayed LOW
    pinMode(pin, INPUT);
    delayMicroseconds(100);

    int reading = digitalRead(pin);

    // If pin is HIGH when we just pulled it LOW, it's shorted to 5V
    if(reading == HIGH) {
      Serial.print("  WARNING: Pin ");
      Serial.print(pin);
      Serial.println(" stuck HIGH (short to 5V!)");
      shortTo5V = true;
    }
  }

  if(!shortTo5V) {
    Serial.println("  ✓ No shorts to 5V detected");
  } else {
    Serial.println();
    Serial.println("  ⚠ DANGER: Shorts to 5V detected!");
    Serial.println("  Disconnect power and check wiring!");
  }

  Serial.println();
  delay(2000);

  // PHASE 2: GND CONNECTION TEST
  Serial.println("PHASE 2: GND connection test");
  Serial.println("Touch DB-50 pins to GND to test");
  Serial.println();

  // Set all test pins to INPUT_PULLUP mode
  for(int pin = 34; pin <= 52; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }

  Serial.println("Ready! Pins configured.");
  Serial.println();
}

void loop() {
  boolean pinDetected = false;
  String pins = "";

  // Scan all pins
  for(int pin = 34; pin <= 52; pin++) {
    // Read pin state (LOW = connected to GND)
    if(digitalRead(pin) == LOW) {
      // Serial.print("Pin ");
      // Serial.print(pin);
      // Serial.println(" detected LOW (connected to GND)");
      pinDetected = true;
      pins = pins + " " + pin;
    }
  }

  // Visual separator if any pins detected
  if(pinDetected) {
    Serial.println(pins);
  }

  // Delay between scans
  delay(500);
}