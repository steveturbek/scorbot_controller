/*
  SCORBOT Base Motor - HOMING SYSTEM with STALL DETECTION (POLLING VERSION)

  This version uses POLLING instead of interrupts for encoder reading,
  since the encoder pins (34, 35) are not interrupt-capable on Arduino Mega.

  Changes from interrupt version:
  - No attachInterrupt() calls
  - updateEncoder() called in main loop instead
  - Higher loop frequency for accurate encoder tracking

  All other functionality identical to base_motor_test.ino
*/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#include "../../scorbot.h"

// ============================================================================
// HOMING PARAMETERS
// ============================================================================

#define HOMING_SEARCH_SPEED 50      // PWM value for initial search (0-255)
#define HOMING_APPROACH_SPEED 30     // PWM value for final approach
#define HOMING_BACKOFF_COUNTS 50     // Encoder counts to back off after finding switch
#define HOMING_TIMEOUT_MS 10000      // Max time to find home switch

// ============================================================================
// STALL DETECTION PARAMETERS
// ============================================================================

#define STALL_CHECK_INTERVAL_MS 50   // How often to check encoder
#define STALL_THRESHOLD_MS 100       // No movement = stall
#define STALL_MIN_ENCODER_CHANGE 2   // Min counts expected in check interval

// ============================================================================
// SOFTWARE LIMITS (will be calibrated during use)
// ============================================================================

#define MAX_POSITION_CW 1500         // Clockwise limit from home
#define MAX_POSITION_CCW -1500       // Counter-clockwise limit from home
#define LIMIT_WARNING_MARGIN 100     // Start slowing this far from limit

// ============================================================================
// STATE MACHINE
// ============================================================================

enum HomingState {
  UNINITIALIZED,
  HOMING_SEARCH_CCW,
  HOMING_SEARCH_CW,
  HOMING_BACKOFF,
  HOMING_APPROACH,
  HOMED,
  FAULT
};

HomingState currentState = UNINITIALIZED;
const char* stateNames[] = {
  "UNINITIALIZED",
  "HOMING_SEARCH_CCW",
  "HOMING_SEARCH_CW",
  "HOMING_BACKOFF",
  "HOMING_APPROACH",
  "HOMED",
  "FAULT"
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Encoder tracking (NOT volatile since we're polling, not using interrupts)
long encoderCount = 0;
int lastEncoded = 0;

// Stall detection
long lastEncoderCount = 0;
unsigned long lastEncoderCheckTime = 0;
int stallCounter = 0;
bool motorActive = false;

// Homing sequence
unsigned long homingStartTime = 0;
long backoffTargetCount = 0;
bool triedCCW = false;
bool triedCW = false;

// Debouncing
unsigned long lastSwitchDebounceTime = 0;
const int SWITCH_DEBOUNCE_MS = 10;

// ============================================================================
// ENCODER READING (POLLED VERSION)
// ============================================================================

void updateEncoder() {
  int MSB = digitalRead(BASE_ENCODER_P0);
  int LSB = digitalRead(BASE_ENCODER_P1);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Quadrature decoding
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void stopMotor() {
  digitalWrite(BASE_MOTOR_DIR1, LOW);
  digitalWrite(BASE_MOTOR_DIR2, LOW);
  analogWrite(BASE_MOTOR_PWM, 0);
  motorActive = false;
}

void moveMotorCW(int speed) {
  digitalWrite(BASE_MOTOR_DIR1, HIGH);
  digitalWrite(BASE_MOTOR_DIR2, LOW);
  analogWrite(BASE_MOTOR_PWM, speed);
  motorActive = true;
}

void moveMotorCCW(int speed) {
  digitalWrite(BASE_MOTOR_DIR1, LOW);
  digitalWrite(BASE_MOTOR_DIR2, HIGH);
  analogWrite(BASE_MOTOR_PWM, speed);
  motorActive = true;
}

// ============================================================================
// SAFETY FUNCTIONS
// ============================================================================

bool isHomeSwitchPressed() {
  if (digitalRead(BASE_SWITCH) == LOW) {
    if (millis() - lastSwitchDebounceTime > SWITCH_DEBOUNCE_MS) {
      lastSwitchDebounceTime = millis();
      return true;
    }
  }
  return false;
}

bool checkStall() {
  if (!motorActive) {
    return false;
  }

  unsigned long currentTime = millis();

  if (currentTime - lastEncoderCheckTime >= STALL_CHECK_INTERVAL_MS) {
    long currentCount = encoderCount;
    long encoderChange = abs(currentCount - lastEncoderCount);

    if (encoderChange < STALL_MIN_ENCODER_CHANGE) {
      stallCounter++;

      if (stallCounter * STALL_CHECK_INTERVAL_MS >= STALL_THRESHOLD_MS) {
        Serial.println("!!! STALL DETECTED !!!");
        stallCounter = 0;
        return true;
      }
    } else {
      stallCounter = 0;
    }

    lastEncoderCount = currentCount;
    lastEncoderCheckTime = currentTime;
  }

  return false;
}

bool isInSafeRange() {
  return (encoderCount >= MAX_POSITION_CCW && encoderCount <= MAX_POSITION_CW);
}

bool isNearLimit() {
  return (encoderCount <= (MAX_POSITION_CCW + LIMIT_WARNING_MARGIN) ||
          encoderCount >= (MAX_POSITION_CW - LIMIT_WARNING_MARGIN));
}

// ============================================================================
// HOMING STATE MACHINE
// ============================================================================

void startHoming() {
  Serial.println("\n========================================");
  Serial.println("STARTING HOMING SEQUENCE");
  Serial.println("========================================");

  stopMotor();
  currentState = HOMING_SEARCH_CCW;
  homingStartTime = millis();
  triedCCW = false;
  triedCW = false;
  stallCounter = 0;

  Serial.println("Searching CCW for home switch...");
  moveMotorCCW(HOMING_SEARCH_SPEED);
}

void updateHomingStateMachine() {
  if (millis() - homingStartTime > HOMING_TIMEOUT_MS) {
    Serial.println("\n!!! HOMING TIMEOUT !!!");
    Serial.println("Failed to find home switch within timeout period");
    currentState = FAULT;
    stopMotor();
    return;
  }

  switch (currentState) {

    case HOMING_SEARCH_CCW:
      if (isHomeSwitchPressed()) {
        Serial.println("Home switch found! (CCW search)");
        stopMotor();
        delay(50);

        backoffTargetCount = encoderCount - HOMING_BACKOFF_COUNTS;
        currentState = HOMING_BACKOFF;
        Serial.print("Backing off to count: ");
        Serial.println(backoffTargetCount);
        moveMotorCW(HOMING_SEARCH_SPEED);
        return;
      }

      if (checkStall()) {
        Serial.println("Stalled in CCW direction, trying CW...");
        stopMotor();
        delay(100);
        triedCCW = true;
        currentState = HOMING_SEARCH_CW;
        moveMotorCW(HOMING_SEARCH_SPEED);
        return;
      }
      break;

    case HOMING_SEARCH_CW:
      if (isHomeSwitchPressed()) {
        Serial.println("Home switch found! (CW search)");
        stopMotor();
        delay(50);

        backoffTargetCount = encoderCount + HOMING_BACKOFF_COUNTS;
        currentState = HOMING_BACKOFF;
        Serial.print("Backing off to count: ");
        Serial.println(backoffTargetCount);
        moveMotorCCW(HOMING_SEARCH_SPEED);
        return;
      }

      if (checkStall()) {
        Serial.println("Stalled in CW direction!");
        if (triedCCW) {
          Serial.println("\n!!! FAULT: Stalled in both directions !!!");
          Serial.println("Possible issues:");
          Serial.println("  - Microswitch not working");
          Serial.println("  - Mechanical obstruction");
          Serial.println("  - Motor not connected properly");
          currentState = FAULT;
          stopMotor();
        } else {
          Serial.println("Trying CCW direction...");
          stopMotor();
          delay(100);
          triedCW = true;
          currentState = HOMING_SEARCH_CCW;
          moveMotorCCW(HOMING_SEARCH_SPEED);
        }
        return;
      }
      break;

    case HOMING_BACKOFF:
      if ((backoffTargetCount > encoderCount && encoderCount <= backoffTargetCount) ||
          (backoffTargetCount < encoderCount && encoderCount >= backoffTargetCount)) {
        Serial.println("Backoff complete, beginning slow approach...");
        stopMotor();
        delay(50);
        currentState = HOMING_APPROACH;

        if (encoderCount < 0) {
          moveMotorCW(HOMING_APPROACH_SPEED);
        } else {
          moveMotorCCW(HOMING_APPROACH_SPEED);
        }
        return;
      }

      if (checkStall()) {
        Serial.println("!!! Stalled during backoff - FAULT !!!");
        currentState = FAULT;
        stopMotor();
        return;
      }
      break;

    case HOMING_APPROACH:
      if (isHomeSwitchPressed()) {
        stopMotor();
        delay(50);

        encoderCount = 0;
        lastEncoderCount = 0;

        Serial.println("\n========================================");
        Serial.println("HOMING COMPLETE!");
        Serial.println("========================================");
        Serial.println("Position set to 0 (home)");
        Serial.print("Software limits: ");
        Serial.print(MAX_POSITION_CCW);
        Serial.print(" to ");
        Serial.println(MAX_POSITION_CW);
        Serial.println("\nReady for operation!");
        Serial.println("Commands: '+' = CW, '-' = CCW, 's' = stop, 'p' = position");
        Serial.println("========================================\n");

        currentState = HOMED;
        return;
      }

      if (checkStall()) {
        Serial.println("!!! Stalled during final approach - FAULT !!!");
        Serial.println("This shouldn't happen. Check mechanics.");
        currentState = FAULT;
        stopMotor();
        return;
      }
      break;

    case HOMED:
    case FAULT:
    case UNINITIALIZED:
      break;
  }
}

// ============================================================================
// MANUAL CONTROL
// ============================================================================

void moveManualCW() {
  if (currentState != HOMED) {
    Serial.println("ERROR: Must home first! Send 'h' to start homing.");
    return;
  }

  if (encoderCount >= MAX_POSITION_CW) {
    Serial.println("ERROR: Already at CW limit!");
    return;
  }

  Serial.println("Moving CW...");
  int speed = isNearLimit() ? (HOMING_SEARCH_SPEED / 2) : 100;
  moveMotorCW(speed);
}

void moveManualCCW() {
  if (currentState != HOMED) {
    Serial.println("ERROR: Must home first! Send 'h' to start homing.");
    return;
  }

  if (encoderCount <= MAX_POSITION_CCW) {
    Serial.println("ERROR: Already at CCW limit!");
    return;
  }

  Serial.println("Moving CCW...");
  int speed = isNearLimit() ? (HOMING_SEARCH_SPEED / 2) : 100;
  moveMotorCCW(speed);
}

void printStatus() {
  Serial.println("\n--- STATUS ---");
  Serial.print("State: ");
  Serial.println(stateNames[currentState]);
  Serial.print("Position: ");
  Serial.print(encoderCount);
  Serial.println(" counts");

  if (currentState == HOMED) {
    Serial.print("Limits: [");
    Serial.print(MAX_POSITION_CCW);
    Serial.print(" to ");
    Serial.print(MAX_POSITION_CW);
    Serial.println("]");

    float percentRange = 100.0 * (encoderCount - MAX_POSITION_CCW) /
                        (MAX_POSITION_CW - MAX_POSITION_CCW);
    Serial.print("Position in range: ");
    Serial.print(percentRange, 1);
    Serial.println("%");
  }

  Serial.print("Switch: ");
  Serial.println(isHomeSwitchPressed() ? "PRESSED" : "not pressed");
  Serial.print("Motor: ");
  Serial.println(motorActive ? "ACTIVE" : "stopped");
  Serial.println("--------------\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("SCORBOT BASE MOTOR - HOMING SYSTEM");
  Serial.println("========================================");
  Serial.println("Firmware version: 1.0 (POLLING)");
  Serial.println("Date: 2026-01-08");
  Serial.println();

  pinMode(BASE_MOTOR_DIR1, OUTPUT);
  pinMode(BASE_MOTOR_DIR2, OUTPUT);
  pinMode(BASE_MOTOR_PWM, OUTPUT);
  stopMotor();
  Serial.println("[OK] Motor pins configured");

  // Encoder pins with pullups (NO interrupts - we're polling)
  pinMode(BASE_ENCODER_P0, INPUT_PULLUP);
  pinMode(BASE_ENCODER_P1, INPUT_PULLUP);
  Serial.println("[OK] Encoder configured (POLLING mode)");

  pinMode(BASE_SWITCH, INPUT_PULLUP);
  Serial.println("[OK] Home switch configured");

  Serial.println();
  Serial.println("========================================");
  Serial.println("READY - Awaiting homing command");
  Serial.println("========================================");
  Serial.println("Send 'h' to start homing sequence");
  Serial.println("Send 'p' to print status");
  Serial.println("========================================\n");

  lastEncoderCheckTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // CRITICAL: Poll encoder every loop for accurate tracking
  updateEncoder();

  // Process serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'h':
      case 'H':
        startHoming();
        break;

      case '+':
        moveManualCW();
        break;

      case '-':
        moveManualCCW();
        break;

      case 's':
      case 'S':
        Serial.println("STOP");
        stopMotor();
        break;

      case 'p':
      case 'P':
        printStatus();
        break;

      case 'r':
      case 'R':
        Serial.println("Resetting to UNINITIALIZED state");
        stopMotor();
        currentState = UNINITIALIZED;
        encoderCount = 0;
        Serial.println("Send 'h' to start homing");
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.print("Unknown command: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println("Commands: h=home, +=CW, -=CCW, s=stop, p=status, r=reset");
        break;
    }
  }

  // Update homing state machine
  if (currentState != UNINITIALIZED && currentState != HOMED && currentState != FAULT) {
    updateHomingStateMachine();
  }

  // Safety checks during normal operation
  if (currentState == HOMED && motorActive) {
    if (!isInSafeRange()) {
      Serial.println("\n!!! SOFTWARE LIMIT REACHED !!!");
      Serial.print("Position: ");
      Serial.println(encoderCount);
      stopMotor();
    }

    if (checkStall()) {
      Serial.println("Stall detected during operation!");
      Serial.println("Possible mechanical obstruction. Stopped for safety.");
      stopMotor();
    }

    if (isHomeSwitchPressed() && motorActive) {
      Serial.println("!!! Home switch pressed during operation !!!");
      Serial.println("This is unexpected. Stopping for safety.");
      stopMotor();
    }
  }

  // Small delay to keep serial responsive
  // Note: We can't delay too much or we'll miss encoder pulses
  delay(1);
}
