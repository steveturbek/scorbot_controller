/*
  MULTI-AXIS EXAMPLE - Demonstrates clean syntax with SCORBOT array

  This shows how using the SCORBOT[] struct array makes code
  much more readable and maintainable.
*/

#include "../../scorbot.h"

// Example: Configure all motors at once
void setup() {
  Serial.begin(9600);
  Serial.println("Scorbot Multi-Axis Configuration Example");
  Serial.println("==========================================\n");

  // Configure all 6 motors in a loop
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Motor control pins
    pinMode(SCORBOT[i].dir1, OUTPUT);
    pinMode(SCORBOT[i].dir2, OUTPUT);
    pinMode(SCORBOT[i].pwm, OUTPUT);

    // Encoder pins
    pinMode(SCORBOT[i].encoder_p0, INPUT_PULLUP);
    pinMode(SCORBOT[i].encoder_p1, INPUT_PULLUP);

    // Home switch
    pinMode(SCORBOT[i].home_switch, INPUT_PULLUP);

    // Stop motor initially
    digitalWrite(SCORBOT[i].dir1, LOW);
    digitalWrite(SCORBOT[i].dir2, LOW);
    analogWrite(SCORBOT[i].pwm, 0);

    Serial.print("[OK] Configured motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(SCORBOT[i].name);
  }

  Serial.println("\n==========================================");
  Serial.println("All motors configured!");
  Serial.println("==========================================\n");

  // Example: Access specific motor by name
  Serial.println("Example 1: Access base motor by enum:");
  Serial.print("  Base PWM pin: ");
  Serial.println(SCORBOT[MOTOR_BASE].pwm);
  Serial.print("  Base name: ");
  Serial.println(SCORBOT[MOTOR_BASE].name);

  Serial.println("\nExample 2: Access by index:");
  Serial.print("  Motor 0 (");
  Serial.print(SCORBOT[0].name);
  Serial.print(") PWM pin: ");
  Serial.println(SCORBOT[0].pwm);

  Serial.println("\nExample 3: Check all home switches:");
  checkAllSwitches();

  Serial.println("\n==========================================");
}

// Example function: Move any motor
void moveMotor(int motorIndex, int speed, bool clockwise) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
    Serial.println("ERROR: Invalid motor index");
    return;
  }

  Serial.print("Moving ");
  Serial.print(SCORBOT[motorIndex].name);
  Serial.print(clockwise ? " CW" : " CCW");
  Serial.print(" at speed ");
  Serial.println(speed);

  if (clockwise) {
    digitalWrite(SCORBOT[motorIndex].dir1, HIGH);
    digitalWrite(SCORBOT[motorIndex].dir2, LOW);
  } else {
    digitalWrite(SCORBOT[motorIndex].dir1, LOW);
    digitalWrite(SCORBOT[motorIndex].dir2, HIGH);
  }
  analogWrite(SCORBOT[motorIndex].pwm, speed);
}

// Example function: Stop any motor
void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  digitalWrite(SCORBOT[motorIndex].dir1, LOW);
  digitalWrite(SCORBOT[motorIndex].dir2, LOW);
  analogWrite(SCORBOT[motorIndex].pwm, 0);
}

// Example function: Check all switches
void checkAllSwitches() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    bool pressed = (digitalRead(SCORBOT[i].home_switch) == LOW);
    Serial.print("  ");
    Serial.print(SCORBOT[i].name);
    Serial.print(" switch: ");
    Serial.println(pressed ? "PRESSED" : "not pressed");
  }
}

// Example function: Read encoder for any motor
void readEncoder(int motorIndex, int &p0, int &p1) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  p0 = digitalRead(SCORBOT[motorIndex].encoder_p0);
  p1 = digitalRead(SCORBOT[motorIndex].encoder_p1);
}

void loop() {
  // Example: Move base motor slowly CW for 2 seconds
  Serial.println("\nTest: Base motor CW");
  moveMotor(MOTOR_BASE, 50, true);
  delay(2000);
  stopMotor(MOTOR_BASE);
  delay(1000);

  // Example: Move shoulder motor slowly CCW for 2 seconds
  Serial.println("Test: Shoulder motor CCW");
  moveMotor(MOTOR_SHOULDER, 50, false);
  delay(2000);
  stopMotor(MOTOR_SHOULDER);
  delay(1000);

  Serial.println("\nChecking all switches:");
  checkAllSwitches();

  Serial.println("\nPausing 5 seconds before next cycle...\n");
  delay(5000);
}
