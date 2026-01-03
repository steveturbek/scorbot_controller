/*
  SCORBOT Base Motor Test Sketch

  This sketch tests the base motor (Motor 1) by:
  - Moving back and forth in increasingly wider sweeps
  - Counting encoder steps
  - Detecting the home microswitch
  - Outputting position data via serial

  Hardware connections:
  - Motor direction: pins 22, 23
  - Motor PWM speed: pin 2
  - Encoder P0: pin 20 (INT3)
  - Encoder P1: pin 21 (INT2)
  - Microswitch: pin 44
*/

// Motor control pins
const int MOTOR_DIR_PIN1 = 22;  // IN1
const int MOTOR_DIR_PIN2 = 23;  // IN2
const int MOTOR_PWM_PIN = 2;    // ENA

// Encoder pins
const int ENCODER_P0 = 20;      // INT3
const int ENCODER_P1 = 21;      // INT2

// Microswitch pin
const int MICROSWITCH_PIN = 44;

// Motor control variables
const int SLOW_SPEED = 80;      // PWM value (0-255), start slow for testing
int motorDirection = 1;         // 1 = clockwise, -1 = counterclockwise

// Encoder variables
volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// Sweep control variables
int sweepNumber = 0;
int maxStepsPerSweep = 100;     // Start with 100 steps
const int SWEEP_INCREMENT = 100; // Add 100 steps each sweep
bool movingForward = true;
long targetPosition = 0;
long startPosition = 0;

// State machine
enum State {
  INIT,
  SWEEPING,
  MICROSWITCH_FOUND,
  STOPPED
};
State currentState = INIT;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }

  Serial.println("SCORBOT Base Motor Test");
  Serial.println("========================");
  Serial.println();

  // Setup motor control pins
  pinMode(MOTOR_DIR_PIN1, OUTPUT);
  pinMode(MOTOR_DIR_PIN2, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // Setup encoder pins with internal pullup resistors
  pinMode(ENCODER_P0, INPUT_PULLUP);
  pinMode(ENCODER_P1, INPUT_PULLUP);

  // Setup microswitch pin with internal pullup
  pinMode(MICROSWITCH_PIN, INPUT_PULLUP);

  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_P0), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_P1), updateEncoder, CHANGE);

  // Stop motor initially
  stopMotor();

  Serial.println("Initialization complete");
  Serial.println("Starting sweep test in 2 seconds...");
  Serial.println();
  delay(2000);

  // Start first sweep
  startPosition = encoderCount;
  sweepNumber = 1;
  targetPosition = startPosition + maxStepsPerSweep;
  movingForward = true;
  currentState = SWEEPING;

  Serial.print("Sweep #");
  Serial.print(sweepNumber);
  Serial.print(" - Target steps: +/-");
  Serial.println(maxStepsPerSweep);

  setMotorDirection(1);  // Start moving forward
  setMotorSpeed(SLOW_SPEED);
}

void loop() {
  // Check microswitch first - safety stop
  if (digitalRead(MICROSWITCH_PIN) == LOW) {  // Active low (pullup)
    if (currentState != MICROSWITCH_FOUND && currentState != STOPPED) {
      stopMotor();
      currentState = MICROSWITCH_FOUND;
      Serial.println();
      Serial.println("*** MICROSWITCH DETECTED! ***");
      Serial.print("Position when found: ");
      Serial.println(encoderCount);
      Serial.println("Motor stopped.");
      Serial.println();
      Serial.println("Test complete - microswitch located successfully!");
    }
    return;  // Stay stopped
  }

  // Normal operation
  switch (currentState) {
    case SWEEPING:
      // Check if we've reached the target position
      if (movingForward && encoderCount >= targetPosition) {
        // Reverse direction
        stopMotor();
        delay(500);  // Brief pause at end of travel

        Serial.print("  Forward complete. Position: ");
        Serial.print(encoderCount);
        Serial.print(" (moved ");
        Serial.print(encoderCount - startPosition);
        Serial.println(" steps)");

        movingForward = false;
        targetPosition = startPosition - maxStepsPerSweep;
        setMotorDirection(-1);  // Reverse
        setMotorSpeed(SLOW_SPEED);

      } else if (!movingForward && encoderCount <= targetPosition) {
        // Completed one full sweep, start next wider sweep
        stopMotor();
        delay(500);  // Brief pause at end of travel

        Serial.print("  Reverse complete. Position: ");
        Serial.print(encoderCount);
        Serial.print(" (moved ");
        Serial.print(startPosition - encoderCount);
        Serial.println(" steps)");
        Serial.println();

        // Prepare next sweep
        sweepNumber++;
        maxStepsPerSweep += SWEEP_INCREMENT;
        startPosition = encoderCount;
        targetPosition = startPosition + maxStepsPerSweep;
        movingForward = true;

        Serial.print("Sweep #");
        Serial.print(sweepNumber);
        Serial.print(" - Target steps: +/-");
        Serial.println(maxStepsPerSweep);

        delay(1000);  // Pause between sweeps

        setMotorDirection(1);  // Forward
        setMotorSpeed(SLOW_SPEED);
      }

      // Print position updates every 50 steps
      static long lastPrintPosition = 0;
      if (abs(encoderCount - lastPrintPosition) >= 50) {
        Serial.print("  Position: ");
        Serial.print(encoderCount);
        Serial.print(" | Target: ");
        Serial.print(targetPosition);
        Serial.print(" | Direction: ");
        Serial.println(movingForward ? "Forward" : "Reverse");
        lastPrintPosition = encoderCount;
      }
      break;

    case STOPPED:
    case MICROSWITCH_FOUND:
      // Do nothing, stay stopped
      break;

    default:
      break;
  }
}

// Set motor direction: 1 = forward, -1 = reverse
void setMotorDirection(int dir) {
  motorDirection = dir;
  if (dir > 0) {
    digitalWrite(MOTOR_DIR_PIN1, HIGH);
    digitalWrite(MOTOR_DIR_PIN2, LOW);
  } else {
    digitalWrite(MOTOR_DIR_PIN1, LOW);
    digitalWrite(MOTOR_DIR_PIN2, HIGH);
  }
}

// Set motor speed (0-255)
void setMotorSpeed(int speed) {
  analogWrite(MOTOR_PWM_PIN, speed);
}

// Stop the motor
void stopMotor() {
  digitalWrite(MOTOR_DIR_PIN1, LOW);
  digitalWrite(MOTOR_DIR_PIN2, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);
}

// Interrupt service routine for encoder
void updateEncoder() {
  int MSB = digitalRead(ENCODER_P0); // Most significant bit
  int LSB = digitalRead(ENCODER_P1); // Least significant bit

  int encoded = (MSB << 1) | LSB;        // Convert to decimal
  int sum = (lastEncoded << 2) | encoded; // Add to previous reading

  // Determine direction based on state change
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;
}
