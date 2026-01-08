/*
  SCORBOT Base Motor - MINIMAL TEST

  This is the absolute simplest test - just spin the motor!
  No encoders, no switches, just motor control.

  Hardware connections:
  - Motor+: pin 22 (to L298N Board 1, Input 1)
  - Motor-: pin 24 (to L298N Board 1, Input 2)
  - Motor PWM speed: pin 2 (to L298N Board 1, ENA)

  L298N Setup:
  - External power (7-12V) to L298N power terminals
  - Motor connected to Motor A output terminals
  - Arduino GND connected to L298N GND
  - Pin 22 → IN1
  - Pin 24 → IN2
  - Pin 2 → ENA (if ENA jumper is removed)
    OR ENA jumper in place (if not using pin 2)
*/

// Motor control pins
const int MOTOR_DIR_PIN1 = 24;  // Motor+ to L298N IN1
const int MOTOR_DIR_PIN2 = 25;  // Motor- to L298N IN2
const int MOTOR_PWM_PIN = 6;    // PWM to L298N ENA

const int MOTOR_SPEED = 254;    // PWM value (0-255), start slow

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port
  }

  Serial.println("SCORBOT Base Motor - MINIMAL TEST");
  Serial.println("===================================");
  Serial.println();

  // Setup motor pins
  pinMode(MOTOR_DIR_PIN1, OUTPUT);
  pinMode(MOTOR_DIR_PIN2, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // Motor off initially
  digitalWrite(MOTOR_DIR_PIN1, LOW);
  digitalWrite(MOTOR_DIR_PIN2, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);

  Serial.println("Motor pins configured");
  Serial.println("Waiting 2 seconds...");
  Serial.println();
  delay(2000);

  Serial.println("Test sequence starting:");
  Serial.println();
}

void loop() {
  // Move forward
  Serial.println("1. Moving FORWARD at speed 100 for 2 seconds...");
  digitalWrite(MOTOR_DIR_PIN1, HIGH);
  digitalWrite(MOTOR_DIR_PIN2, LOW);
  analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
  delay(2000);

  // Stop
  Serial.println("   STOP");
  digitalWrite(MOTOR_DIR_PIN1, LOW);
  digitalWrite(MOTOR_DIR_PIN2, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);
  delay(1000);

  // Move reverse
  Serial.println("2. Moving REVERSE at speed 100 for 2 seconds...");
  digitalWrite(MOTOR_DIR_PIN1, LOW);
  digitalWrite(MOTOR_DIR_PIN2, HIGH);
  analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
  delay(2000);

  // Stop
  Serial.println("   STOP");
  digitalWrite(MOTOR_DIR_PIN1, LOW);
  digitalWrite(MOTOR_DIR_PIN2, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);
  delay(1000);

  Serial.println("3. Pausing 3 seconds before next cycle...");
  Serial.println();
  delay(3000);

  // Loop repeats
}
