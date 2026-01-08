/*
  MOTOR ONLY TEST - Check if motor spins without encoder checking

  This will spin the motor without any encoder or stall detection.
  Watch if the motor actually rotates.

  Connections:
  - Motor CCW (DIR1): pin 22
  - Motor CW (DIR2): pin 23
  - Motor PWM: pin 7
*/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#include "/Users/steveturbek/Documents/scorbot_controller/scorbot_controller/scorbot.h"

void setup() {
  Serial.begin(9600);

  // Setup the base motor pins
  setupMotor(MOTOR_BASE);

  Serial.println("Motor Only Test - Base Motor");
  Serial.println("Motor should spin CW for 3 sec, stop, then CCW for 3 sec");
  Serial.println();
}

void loop() {
  Serial.println("Spinning CW at 25% speed...");
  moveMotor(MOTOR_BASE, 25, true);  // CW at 25% speed (0-99 range)
  delay(3000);

  Serial.println("STOP");
  stopMotor(MOTOR_BASE);
  delay(1000);

  Serial.println("Spinning CCW at 25% speed...");
  moveMotor(MOTOR_BASE, 25, false);  // CCW at 25% speed
  delay(3000);

  Serial.println("STOP");
  stopMotor(MOTOR_BASE);
  delay(1000);
}
