/*
  SCORBOT Base Motor - HOMING SYSTEM with RANGE CALIBRATION (POLLING VERSION)

The goal for this program is for me to learn but also to write and test functions for scorbot.h to use later.

  This version uses POLLING instead of interrupts for encoder reading.
  All encoder pins are polled in the main loop for accurate tracking.

  This implements a complete homing system with:
  - Encoder-based position tracking (polled, not interrupt-driven)
  - Microswitch home position detection
  - Stall detection to prevent damage
  - Automatic range calibration (finds CW and CCW limits from home)
  - Software limits after homing
  - Bidirectional search algorithm

  Hardware connections (from scorbot.h):
  - Motor Control (L298N Board 1):
    - Motor DIR1 (CCW): pin 23 (IN1)
    - Motor DIR2 (CW): pin 22 (IN2)
    - Motor PWM: pin 7 (ENA)

  - Encoder (Base Motor):
    - Encoder P0: pin 34 (not interrupt-capable)
    - Encoder P1: pin 35 (not interrupt-capable)

  - Home Microswitch:
    - Base switch: pin 46 (active LOW with internal pullup)

  Serial Commands (9600 baud):
  - 'h' = Start homing sequence
  - '+' = Move CW (after homed)
  - '-' = Move CCW (after homed)
  - 's' = Stop motor
  - 'p' = Print current position and status
  - 'r' = Reset to unhomed state
*/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#include <Arduino.h>
#include "/Users/steveturbek/Documents/scorbot_controller/scorbot_controller/scorbot.h"

// ============================================================================
// HOMING PARAMETERS
// ============================================================================

#define HOMING_SEARCH_SPEED 200      // PWM value for initial search (0-255)
#define HOMING_APPROACH_SPEED 30     // PWM value for final approach
#define HOMING_BACKOFF_COUNTS 50     // Encoder counts to back off after finding switch
#define HOMING_TIMEOUT_MS 100000      // Max time to find home switch

// ============================================================================
// STALL DETECTION PARAMETERS
// ============================================================================

#define STALL_CHECK_INTERVAL_MS 50   // How often to check encoder
#define STALL_THRESHOLD_MS 100       // No movement = stall
#define STALL_MIN_ENCODER_CHANGE 2   // Min counts expected in check interval

// ============================================================================
// SOFTWARE LIMITS (will be calibrated during use)
// ============================================================================

// Base motor: 310° range, assuming ~1500 counts from center to each limit
// These are conservative estimates - calibrate after first homing!
#define MAX_POSITION_CW 1500         // Clockwise limit from home
#define MAX_POSITION_CCW -1500       // Counter-clockwise limit from home
#define LIMIT_WARNING_MARGIN 100     // Start slowing this far from limit

// ============================================================================
// STATE MACHINE
// ============================================================================

enum HomingState {
  UNINITIALIZED,    // Power-on state, position unknown
  HOMING_SEARCH_CCW, // Searching CCW for home switch
  HOMING_SEARCH_CW,  // Searching CW for home switch (if CCW stalled)
  HOMING_BACKOFF,    // Back away from switch slightly
  HOMING_APPROACH,   // Slow final approach for accuracy
  CALIBRATING_CW_RANGE,   // Move CW until stall to find max range
  CALIBRATING_CCW_RANGE,  // Move CCW until stall to find max range
  RETURN_TO_HOME,    // Returning to home switch after calibration
  HOMED,             // Home found, position = 0, ready for operation
  FAULT              // Error state
};

HomingState currentState = UNINITIALIZED;
const char* stateNames[] = {
  "UNINITIALIZED",
  "HOMING_SEARCH_CCW",
  "HOMING_SEARCH_CW",
  "HOMING_BACKOFF",
  "HOMING_APPROACH",
  "CALIBRATING_CW_RANGE",
  "CALIBRATING_CCW_RANGE",
  "RETURN_TO_HOME",
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

// Range calibration
long maxCWFromHome = 0;
long maxCCWFromHome = 0;
unsigned long lastEncoderPrintTime = 0;
const unsigned long ENCODER_PRINT_INTERVAL_MS = 100;  // Print every 100ms
unsigned long returnToHomeStartTime = 0;

// Debouncing
unsigned long lastSwitchDebounceTime = 0;
const int SWITCH_DEBOUNCE_MS = 10;

// ============================================================================
// ENCODER READING (POLLED VERSION)
// ============================================================================

void updateEncoder() {
  int MSB = digitalRead(SCORBOT_REF[MOTOR_BASE].encoder_p0_pin);
  int LSB = digitalRead(SCORBOT_REF[MOTOR_BASE].encoder_p1_pin);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Quadrature decoding: detect direction from state transitions
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
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CCW_pin, LOW);
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CW_pin, LOW);
  analogWrite(SCORBOT_REF[MOTOR_BASE].pwm_pin, 0);
  motorActive = false;
}

void moveMotorCW(int speed) {
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CW_pin, HIGH);
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CCW_pin, LOW);
  analogWrite(SCORBOT_REF[MOTOR_BASE].pwm_pin, speed);
  motorActive = true;
}

void moveMotorCCW(int speed) {
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CCW_pin, HIGH);
  digitalWrite(SCORBOT_REF[MOTOR_BASE].CW_pin, LOW);
  analogWrite(SCORBOT_REF[MOTOR_BASE].pwm_pin, speed);
  motorActive = true;
}

// ============================================================================
// SAFETY FUNCTIONS
// ============================================================================

bool isHomeSwitchPressed() {
  // Active LOW switch with pullup
  if (digitalRead(SCORBOT_REF[MOTOR_BASE].home_switch_pin) == LOW) {
    // Simple debounce
    if (millis() - lastSwitchDebounceTime > SWITCH_DEBOUNCE_MS) {
      lastSwitchDebounceTime = millis();
      return true;
    }
  }
  return false;
}

bool checkStall() {
  // Only check for stall if motor is active
  if (!motorActive) {
    return false;
  }

  unsigned long currentTime = millis();

  // Check at regular intervals
  if (currentTime - lastEncoderCheckTime >= STALL_CHECK_INTERVAL_MS) {
    long currentCount = encoderCount;
    long encoderChange = abs(currentCount - lastEncoderCount);

    // If encoder hasn't moved enough, increment stall counter
    if (encoderChange < STALL_MIN_ENCODER_CHANGE) {
      stallCounter++;

      // If stalled for STALL_THRESHOLD_MS, declare stall
      if (stallCounter * STALL_CHECK_INTERVAL_MS >= STALL_THRESHOLD_MS) {
        Serial.println("STALL DETECTED");
        stallCounter = 0;
        return true;
      }
    } else {
      // Movement detected, reset counter
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
  Serial.println("STARTING HOMING SEQUENCE");

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
  // Check for timeout
  if (millis() - homingStartTime > HOMING_TIMEOUT_MS) {
    Serial.println("Failed to find home switch within timeout period");
    currentState = FAULT;
    stopMotor();
    return;
  }

  switch (currentState) {

    case HOMING_SEARCH_CCW:
      // Check if home switch found
      if (isHomeSwitchPressed()) {
        Serial.println("Home switch found! (CCW search)");
        stopMotor();
        delay(50);  // Let motor stop

        // Prepare for backoff
        backoffTargetCount = encoderCount - HOMING_BACKOFF_COUNTS;
        currentState = HOMING_BACKOFF;
        Serial.print("Backing off to count: ");
        Serial.println(backoffTargetCount);
        moveMotorCW(HOMING_SEARCH_SPEED);
        return;
      }

      // Check for stall
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
      // Check if home switch found
      if (isHomeSwitchPressed()) {
        Serial.println("Home switch found! (CW search)");
        stopMotor();
        delay(50);

        // Prepare for backoff
        backoffTargetCount = encoderCount + HOMING_BACKOFF_COUNTS;
        currentState = HOMING_BACKOFF;
        Serial.print("Backing off to count: ");
        Serial.println(backoffTargetCount);
        moveMotorCCW(HOMING_SEARCH_SPEED);
        return;
      }

      // Check for stall
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
          // Haven't tried CCW yet, try it
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
      // Check if we've backed off enough
      if ((backoffTargetCount > encoderCount && encoderCount <= backoffTargetCount) ||
          (backoffTargetCount < encoderCount && encoderCount >= backoffTargetCount)) {
        Serial.println("Backoff complete, beginning slow approach...");
        stopMotor();
        delay(50);
        currentState = HOMING_APPROACH;

        // Determine approach direction based on where we are
        if (encoderCount < 0) {
          moveMotorCW(HOMING_APPROACH_SPEED);
        } else {
          moveMotorCCW(HOMING_APPROACH_SPEED);
        }
        return;
      }

      // Check for stall during backoff (shouldn't happen, but safety)
      if (checkStall()) {
        Serial.println("!!! Stalled during backoff - FAULT !!!");
        currentState = FAULT;
        stopMotor();
        return;
      }
      break;

    case HOMING_APPROACH:
      // Check if home switch found
      if (isHomeSwitchPressed()) {
        stopMotor();
        delay(50);

        // Set this as home position (zero)
        encoderCount = 0;
        lastEncoderCount = 0;

        Serial.println("HOME POSITION FOUND!");
        Serial.println("Position set to 0 (home)");
        Serial.println("Starting range calibration...");
        Serial.println("Moving CW to find maximum range from home");

        // Start calibration
        currentState = CALIBRATING_CW_RANGE;
        stallCounter = 0;
        lastEncoderPrintTime = millis();
        moveMotorCW(HOMING_SEARCH_SPEED);
        return;
      }

      // Check for stall during approach
      if (checkStall()) {
        Serial.println("!!! Stalled during final approach - FAULT !!!");
        Serial.println("This shouldn't happen. Check mechanics.");
        currentState = FAULT;
        stopMotor();
        return;
      }
      break;

    case CALIBRATING_CW_RANGE:
      // Print encoder count periodically
      // if (millis() - lastEncoderPrintTime >= ENCODER_PRINT_INTERVAL_MS) {
      //  Serial.print("CW Range: " + encoderCount +" counts from home");
      //   lastEncoderPrintTime = millis();
      // }

      // Check for stall (reached limit)
      if (checkStall()) {
        maxCWFromHome = encoderCount;
        stopMotor();
        delay(100);

        Serial.print("MAX_CW_FROM_HOME: ");
        Serial.print(maxCWFromHome);
        Serial.println(" counts");
        Serial.println("\nReturning to home position...");

        // Return to home
        currentState = CALIBRATING_CCW_RANGE;
        stallCounter = 0;
        lastEncoderPrintTime = millis();

        // Move CCW back past home, then search CCW for limit
        Serial.println("Moving CCW to find maximum range from home");
        moveMotorCCW(HOMING_SEARCH_SPEED);
        return;
      }
      break;

    case CALIBRATING_CCW_RANGE:
      // Print encoder count periodically
      // if (millis() - lastEncoderPrintTime >= ENCODER_PRINT_INTERVAL_MS) {
      //   Serial.print("CCW Range: " + encoderCount + " counts from home");
      //   lastEncoderPrintTime = millis();
      // }

      // Check for stall (reached limit)
      if (checkStall()) {
        maxCCWFromHome = encoderCount;
        stopMotor();
        delay(100);

        Serial.print("MAX_CCW_FROM_HOME: ");
        Serial.print(maxCCWFromHome);
        Serial.println(" counts");
        Serial.println("CALIBRATION COMPLETE!");
        Serial.print("CW Limit:  ");
        Serial.print(maxCWFromHome);
        Serial.println(" counts from home");
        Serial.print("CCW Limit: ");
        Serial.print(maxCCWFromHome);
        Serial.println(" counts from home");
        Serial.print("Total Range: ");
        Serial.print(abs(maxCWFromHome - maxCCWFromHome));
        Serial.println(" encoder counts");
        Serial.println("\nReturning to home position...");
        Serial.print("Current position: ");
        Serial.print(encoderCount);
        Serial.println(" counts");

        // Transition to return to home state
        currentState = RETURN_TO_HOME;
        returnToHomeStartTime = millis();
        stallCounter = 0;
        lastEncoderPrintTime = millis();

        // Determine which direction to move based on current position
        if (abs(encoderCount) > 50) {  // Only move if we're far from home
          if (encoderCount > 0) {
            // We're on the positive side, need to go opposite direction
            Serial.println("Moving CCW toward home...");
            moveMotorCCW(HOMING_SEARCH_SPEED);
          } else {
            // We're on the negative side, need to go opposite direction
            Serial.println("Moving CW toward home...");
            moveMotorCW(HOMING_SEARCH_SPEED);
          }
        } else {
          // Already at home, skip to HOMED state
          Serial.println("Already at home position!");
          encoderCount = 0;
          currentState = HOMED;
        }
        return;
      }
      break;

    case RETURN_TO_HOME:
      // Print position periodically
      // if (millis() - lastEncoderPrintTime >= ENCODER_PRINT_INTERVAL_MS) {
      //   Serial.print("Returning to home - Position: ");
      //   Serial.println(encoderCount);
      //   lastEncoderPrintTime = millis();
      // }

      // Check if we've reached home switch
      if (isHomeSwitchPressed()) {
        stopMotor();
        delay(50);
        encoderCount = 0;  // Reset to home

        Serial.println("Home switch reached!");
        Serial.println("\n========================================");
        Serial.println("READY FOR OPERATION!");
        Serial.println("========================================");
        Serial.println("Commands: '+' = CW, '-' = CCW, 's' = stop, 'p' = position");
        Serial.println("========================================\n");

        currentState = HOMED;
        return;
      }

      // Check for stall (hit something or wrong direction)
      if (checkStall()) {
        Serial.println("\n!!! STALL while returning to home !!!");
        Serial.print("Current position: ");
        Serial.println(encoderCount);
        Serial.println("This likely means the motor is moving in the wrong direction.");
        Serial.println("Reversing direction...");

        stopMotor();
        delay(100);
        stallCounter = 0;

        // Reverse direction
        if (encoderCount > 0) {
          Serial.println("Trying CW direction...");
          moveMotorCW(HOMING_SEARCH_SPEED);
        } else {
          Serial.println("Trying CCW direction...");
          moveMotorCCW(HOMING_SEARCH_SPEED);
        }
        return;
      }

      // Timeout safety
      if (millis() - returnToHomeStartTime > 30000) {
        Serial.println("\n!!! TIMEOUT returning to home !!!");
        Serial.println("Failed to find home switch within 30 seconds.");
        stopMotor();
        currentState = FAULT;
        return;
      }
      break;

    case HOMED:
      // Normal operation - handled in main loop
      break;

    case FAULT:
      // Stay stopped
      break;

    case UNINITIALIZED:
      // Waiting for user to start homing
      break;
  }
}

// ============================================================================
// MANUAL CONTROL (after homing)
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

  // Reduce speed if near limit
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

  // Reduce speed if near limit
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
    ; // Wait for serial port
  }


  // Setup base motor using scorbot.h helper function
  // This configures motor pins, encoder pins, and home switch
  setupMotor(MOTOR_BASE);
  stopMotor();

  Serial.println("\n\n========================================");
  Serial.println("SCORBOT BASE MOTOR TEST");
  Serial.println("Send 'h' to start homing sequence");
  Serial.println("Send 'p' to print status");
  Serial.println();

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
      case '=':
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
        // Ignore newlines
        break;

      default:
        Serial.print("Unknown command: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println("Commands: h=home, +=CW, -=CCW, s=stop, p=status, r=reset");
        break;
    }
  }

  // Update homing state machine (includes calibration states)
  if (currentState != UNINITIALIZED && currentState != HOMED && currentState != FAULT) {
    updateHomingStateMachine();
  }

  // Safety checks during normal operation
  if (currentState == HOMED && motorActive) {
    // Check software limits
    if (!isInSafeRange()) {
      Serial.println("\n!!! SOFTWARE LIMIT REACHED !!!");
      Serial.print("Position: ");
      Serial.println(encoderCount);
      stopMotor();
    }

    // Check for stall
    if (checkStall()) {
      Serial.println("Stall detected during operation!");
      Serial.println("Possible mechanical obstruction. Stopped for safety.");
      stopMotor();

      // Could add auto-recovery here: back off slightly and retry
    }

    // Emergency stop on home switch (shouldn't happen in normal use)
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
