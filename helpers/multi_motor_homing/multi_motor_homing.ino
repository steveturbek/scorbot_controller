/*
  SCORBOT Multi-Motor Homing System (Goal-Based Architecture)

  Uses a goal-based approach where each motor pursues high-level goals:
  - GOAL_IDLE: At rest
  - GOAL_FIND_HOME: Locate home switch and set position to 0
  - GOAL_CALIBRATE_RANGE: Find CW/CCW limits and return to home
  - GOAL_MOVE_TO: Move to target position (future)

  Progress is inferred from observable state (encoderCount, motorPWM,
  maxEncoderSteps, home switch) rather than tracking detailed sub-states.

  Serial Commands (9600 baud):
  - 'h' = Start homing ALL motors (find home + calibrate)
  - '0'-'5' = Select motor (0=base, 1=shoulder, etc.)
  - 'f' = Find home for selected motor only
  - 'c' = Calibrate range for selected motor (must be homed first)
  - '+' = Move selected motor CW
  - '-' = Move selected motor CCW
  - 's' = Stop selected motor
  - 'S' = STOP ALL motors (emergency)
  - 'p' = Print status for all motors
  - 'r' = Reset all motors to GOAL_IDLE
*/

#include <Arduino.h>
#include "/Users/steveturbek/Documents/scorbot_controller/scorbot_controller/scorbot.h"

// Currently selected motor (-1 = none)
int selectedMotor = -1;

// Status printing
unsigned long lastStatusPrintTime = 0;
const unsigned long STATUS_PRINT_INTERVAL = 5000;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println("\n========================================");
  Serial.println("SCORBOT MULTI-MOTOR HOMING SYSTEM");
  
  // Initialize hardware
  setupAllMotors();
  initializeAllJointStates();

  // Safety: stop all motors
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    stopMotor(i);
  }

  Serial.println("\nCommands:");
  Serial.println("  h       = Home all (find + calibrate)");
  Serial.println("  0-5     = Select motor");
  Serial.println("  f       = Find home (selected motor)");
  Serial.println("  c       = Calibrate (selected motor)");
  Serial.println("  +/-     = Jog selected motor");
  Serial.println("  s/S     = Stop selected/all");
  Serial.println("  p       = Print status (all motors)");
  Serial.println("  d       = Debug state (selected motor)");
  Serial.println("  r       = Reset all to idle");
  Serial.println("========================================\n");

  // Initialize encoder check time
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    jointState[i].lastEncoderCheckTime = millis();
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // CRITICAL: Update all encoders every loop
  updateAllEncoders();

  // Process serial commands
  processSerialCommands();

  // Update all active goals
  updateAllGoals();

  // Safety monitoring
  performSafetyChecks();

  // Periodic status
  if (millis() - lastStatusPrintTime >= STATUS_PRINT_INTERVAL) {
    printActiveGoals();
    lastStatusPrintTime = millis();
  }

  delay(1);
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'h':
      case 'H':
        Serial.println("Starting homing sequence for all motors");
        startHomingAll();
        break;

      case '0': case '1': case '2':
      case '3': case '4': case '5':
        selectedMotor = cmd - '0';
        Serial.print("Selected: ");
        Serial.println(SCORBOT_REF[selectedMotor].name);
        break;

      case 'f':
      case 'F':
        if (selectedMotor >= 0) {
          Serial.print("Finding home for ");
          Serial.println(SCORBOT_REF[selectedMotor].name);
          setGoal(selectedMotor, GOAL_FIND_HOME);
        } else {
          Serial.println("No motor selected");
        }
        break;

      case 'c':
      case 'C':
        if (selectedMotor >= 0) {
          if (jointState[selectedMotor].hasFoundHome) {
            Serial.print("Calibrating ");
            Serial.println(SCORBOT_REF[selectedMotor].name);
            setGoal(selectedMotor, GOAL_CALIBRATE_RANGE);
          } else {
            Serial.println("Must find home first (press 'f')");
          }
        } else {
          Serial.println("No motor selected");
        }
        break;

      case '+':
      case '=':
        moveSelectedMotorCW();
        break;

      case '-':
      case '_':
        moveSelectedMotorCCW();
        break;

      case 's':
        if (selectedMotor >= 0) {
          stopMotor(selectedMotor);
          setGoal(selectedMotor, GOAL_IDLE);
        }
        break;

      case 'S':
        Serial.println("EMERGENCY STOP ALL");
        for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
          stopMotor(i);
          setGoal(i, GOAL_IDLE);
        }
        break;

      case 'p':
      case 'P':
        printFullStatus();
        break;

      case 'd':
      case 'D':
        if (selectedMotor >= 0) {
          printDetailedState(selectedMotor);
        } else {
          Serial.println("No motor selected");
        }
        break;

      case 'r':
      case 'R':
        Serial.println("Resetting all to IDLE");
        for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
          stopMotor(i);
          setGoal(i, GOAL_IDLE);
        }
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.print("Unknown: '");
        Serial.print(cmd);
        Serial.println("'");
        break;
    }
  }
}

// ============================================================================
// GOAL UPDATE DISPATCHER
// ============================================================================

// Update all active goals
void updateAllGoals() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    if (jointState[i].currentGoal != GOAL_IDLE &&
        jointState[i].currentGoal != GOAL_FAULT) {
      updateGoal(i);
    }
  }
}

void updateGoal(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;

  MotorGoal goal = jointState[motorIndex].currentGoal;

  // Timeout check
  if (goal != GOAL_IDLE && goal != GOAL_FAULT) {
    if (millis() - jointState[motorIndex].goalStartTime > GOAL_TIMEOUT_MS) {
      Serial.print("TIMEOUT: ");
      Serial.println(SCORBOT_REF[motorIndex].name);
      stopMotor(motorIndex);
      setGoal(motorIndex, GOAL_FAULT);
      return;
    }
  }

  switch (goal) {
    case GOAL_FIND_HOME:
      updateGoalFindHome(motorIndex);
      break;

    case GOAL_CALIBRATE_RANGE:
      updateGoalCalibrateRange(motorIndex);
      break;

    case GOAL_MOVE_TO:
      updateGoalMoveTo(motorIndex);
      break;

    case GOAL_IDLE:
    case GOAL_FAULT:
      // Nothing to do
      break;
  }
}

// ============================================================================
// GOAL: FIND_HOME
// ============================================================================

void updateGoalFindHome(int motorIndex) {
  // Found home switch
  if (isHomeSwitchPressed(motorIndex)) {
    if (!jointState[motorIndex].backingOff) {
      // First contact - back off
      stopMotor(motorIndex);
      delay(50);
      jointState[motorIndex].backoffTargetCount =
        jointState[motorIndex].encoderCount - HOMING_BACKOFF_COUNTS;
      jointState[motorIndex].backingOff = true;
      moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
      return;
    } else {
      // Final approach complete - home found!
      stopMotor(motorIndex);
      jointState[motorIndex].encoderCount = 0;
      jointState[motorIndex].hasFoundHome = true;
      setGoal(motorIndex, GOAL_CALIBRATE_RANGE); // Auto-start calibration
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Home found. Starting calibration...");
      return;
    }
  }

  // Backing off from switch
  if (jointState[motorIndex].backingOff) {
    if (jointState[motorIndex].encoderCount >=
        jointState[motorIndex].backoffTargetCount) {
      // Backoff complete, slow approach
      stopMotor(motorIndex);
      delay(50);
      moveMotorCCW(motorIndex, HOMING_APPROACH_SPEED);
    }
    return;
  }

  // Still searching
  if (!jointState[motorIndex].motorActive) {
    // Start search (try CCW first)
    moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
    jointState[motorIndex].searchedCCW = true;
    return;
  }

  // Check for stall (hit limit)
  if (checkStall(motorIndex)) {
    stopMotor(motorIndex);
    delay(100);

    if (jointState[motorIndex].searchedCCW && !jointState[motorIndex].searchedCW) {
      // Try CW
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Trying CW");
      jointState[motorIndex].searchedCW = true;
      moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
    } else if (jointState[motorIndex].searchedCW && !jointState[motorIndex].searchedCCW) {
      // Try CCW
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Trying CCW");
      jointState[motorIndex].searchedCCW = true;
      moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
    } else {
      // Both directions stalled - FAULT
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": FAULT - stalled both directions");
      setGoal(motorIndex, GOAL_FAULT);
    }
  }
}

// ============================================================================
// GOAL: CALIBRATE_RANGE
// ============================================================================

void updateGoalCalibrateRange(int motorIndex) {
  // Must have home first
  if (!jointState[motorIndex].hasFoundHome) {
    Serial.println("Error: Need home before calibrating");
    setGoal(motorIndex, GOAL_FAULT);
    return;
  }

  // Phase 1: Find CW limit
  if (jointState[motorIndex].maxEncoderStepsFromHomeCW == 0) {
    if (!jointState[motorIndex].motorActive) {
      moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
    }

    if (checkStall(motorIndex)) {
      jointState[motorIndex].maxEncoderStepsFromHomeCW =
        jointState[motorIndex].encoderCount;
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.print(": CW limit = ");
      Serial.println(jointState[motorIndex].maxEncoderStepsFromHomeCW);
      stopMotor(motorIndex);
      delay(100);
    }
    return;
  }

  // Phase 2: Find CCW limit
  if (jointState[motorIndex].maxEncoderStepsFromHomeCCW == 0) {
    if (!jointState[motorIndex].motorActive) {
      moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
    }

    if (checkStall(motorIndex)) {
      jointState[motorIndex].maxEncoderStepsFromHomeCCW =
        jointState[motorIndex].encoderCount;
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.print(": CCW limit = ");
      Serial.println(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
      stopMotor(motorIndex);
      delay(100);
    }
    return;
  }

  // Phase 3: Return to home
  if (abs(jointState[motorIndex].encoderCount) > HOME_POSITION_TOLERANCE) {
    if (!jointState[motorIndex].motorActive) {
      // Move toward home
      if (jointState[motorIndex].encoderCount > 0) {
        moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
      } else {
        moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
      }
    }

    if (isHomeSwitchPressed(motorIndex)) {
      stopMotor(motorIndex);
      jointState[motorIndex].encoderCount = 0;
      setGoal(motorIndex, GOAL_IDLE);
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.print(": Calibration complete. Range: [");
      Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
      Serial.print(" to ");
      Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCW);
      Serial.println("]");
    }
    return;
  }

  // Already at home - done!
  setGoal(motorIndex, GOAL_IDLE);
  Serial.print(SCORBOT_REF[motorIndex].name);
  Serial.print(": Calibration complete. Range: [");
  Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
  Serial.print(" to ");
  Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCW);
  Serial.println("]");
}

// ============================================================================
// GOAL: MOVE_TO (placeholder for future)
// ============================================================================

void updateGoalMoveTo(int motorIndex) {
  long target = jointState[motorIndex].targetPosition;
  long current = jointState[motorIndex].encoderCount;
  long error = target - current;

  // Deadband: stop if within 10 counts of target to prevent infinite loop
  if (abs(error) < 10) {
    stopMotor(motorIndex);
    setGoal(motorIndex, GOAL_IDLE);
    Serial.print(SCORBOT_REF[motorIndex].name);
    Serial.print(": stopped at: ");
    Serial.print(current);
    Serial.print(", ");
    Serial.print(error);
    Serial.println(" steps away from target: ");
    Serial.println(target);
    return;
  }

  // Proportional control with minimum speed threshold
  int speed;
  if (abs(error) > 100) {
    speed = 200;  // Full speed when far away
  } else if (abs(error) > 50) {
    speed = 150;  // Medium speed when getting close
  } else {
    speed = 100;  // Slow speed for final approach
  }

  if (error > 0) {
    moveMotorCW(motorIndex, speed);
  } else {
    moveMotorCCW(motorIndex, speed);
  }
}

// ============================================================================
// SAFETY CHECKS
// ============================================================================

void performSafetyChecks() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    // Don't do safety checks when idle, faulted, or actively calibrating
    if (jointState[i].currentGoal == GOAL_IDLE ||
        jointState[i].currentGoal == GOAL_FAULT ||
        jointState[i].currentGoal == GOAL_CALIBRATE_RANGE) {
      continue;
    }

    // Check limits (only during normal operation, not during calibration)
    if (isCalibrated(i) && !isInSafeRange(i)) {
      Serial.print("LIMIT REACHED: ");
      Serial.println(SCORBOT_REF[i].name);
      stopMotor(i);
      setGoal(i, GOAL_IDLE);
    }
  }
}

// ============================================================================
// STATUS PRINTING
// ============================================================================

void printFullStatus() {
  Serial.println("\nMOTOR STATUS");

  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    Serial.print(SCORBOT_REF[i].name);
    Serial.print(": ");
    Serial.print(MOTOR_GOAL_NAMES[jointState[i].currentGoal]);
    Serial.print(" | Pos: ");
    Serial.print(jointState[i].encoderCount);

    if (jointState[i].hasFoundHome) {
      Serial.print(" [HOMED]");
    }

    if (isCalibrated(i)) {
      Serial.print(" | Range: [");
      Serial.print(jointState[i].maxEncoderStepsFromHomeCCW);
      Serial.print(" to ");
      Serial.print(jointState[i].maxEncoderStepsFromHomeCW);
      Serial.print("]");
    }

    Serial.println();
  }
  Serial.println("");
}

void printDetailedState(int motorIndex) {
  Serial.println("\n========================================");
  Serial.print("DETAILED STATE: ");
  Serial.println(SCORBOT_REF[motorIndex].name);

  // Current goal and position
  Serial.print("Goal: ");
  Serial.println(MOTOR_GOAL_NAMES[jointState[motorIndex].currentGoal]);
  Serial.print("Encoder Count: ");
  Serial.println(jointState[motorIndex].encoderCount);
  Serial.print("Target Position: ");
  Serial.println(jointState[motorIndex].targetPosition);

  // Motor state
  Serial.print("Motor PWM: ");
  Serial.println(jointState[motorIndex].motorPWM);
  Serial.print("Motor Active: ");
  Serial.println(jointState[motorIndex].motorActive ? "YES" : "NO");

  // Home switch state (live reading)
  Serial.print("Home Switch Pressed: ");
  Serial.println(isHomeSwitchPressed(motorIndex) ? "YES" : "NO");
  Serial.print("Home Switch Pin: ");
  Serial.print(SCORBOT_REF[motorIndex].home_switch_pin);
  Serial.print(" = ");
  Serial.println(digitalRead(SCORBOT_REF[motorIndex].home_switch_pin) == LOW ? "LOW (pressed)" : "HIGH (not pressed)");

  // Calibration limits
  Serial.print("CW Limit: ");
  Serial.println(jointState[motorIndex].maxEncoderStepsFromHomeCW);
  Serial.print("CCW Limit: ");
  Serial.println(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
  Serial.print("Calibrated: ");
  Serial.println(isCalibrated(motorIndex) ? "YES" : "NO");

  // Progress flags
  Serial.print("Has Found Home: ");
  Serial.println(jointState[motorIndex].hasFoundHome ? "YES" : "NO");
  Serial.print("Backing Off: ");
  Serial.println(jointState[motorIndex].backingOff ? "YES" : "NO");
  Serial.print("Searched CCW: ");
  Serial.println(jointState[motorIndex].searchedCCW ? "YES" : "NO");
  Serial.print("Searched CW: ");
  Serial.println(jointState[motorIndex].searchedCW ? "YES" : "NO");

  // Timing
  Serial.print("Goal Runtime: ");
  Serial.print(millis() - jointState[motorIndex].goalStartTime);
  Serial.println(" ms");
  Serial.print("Stall Counter: ");
  Serial.println(jointState[motorIndex].stallCounter);

}

void printActiveGoals() {
  bool anyActive = false;
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    if (jointState[i].currentGoal != GOAL_IDLE &&
        jointState[i].currentGoal != GOAL_FAULT) {
      if (!anyActive) {
        Serial.println("--- Active Goals ---");
        anyActive = true;
      }
      Serial.print("  ");
      Serial.print(SCORBOT_REF[i].name);
      Serial.print(": ");
      Serial.println(MOTOR_GOAL_NAMES[jointState[i].currentGoal]);
    }
  }
}

// ============================================================================
// MANUAL CONTROL
// ============================================================================


void moveSelectedMotorCW() {
  if (selectedMotor < 0) {
    Serial.println("No motor selected");
    return;
  }

  if (!jointState[selectedMotor].hasFoundHome) {
    Serial.println("Must home first");
    return;
  }

  // Check if already at or near CW limit
  if (isCalibrated(selectedMotor)) {
    long nextPosition = jointState[selectedMotor].encoderCount + JOG_STEP_SIZE;
    if (nextPosition >= jointState[selectedMotor].maxEncoderStepsFromHomeCW) {
      Serial.println("Cannot jog: at CW limit");
      return;
    }
  }

  // Set goal to move to current position + jog step
  jointState[selectedMotor].targetPosition =
    jointState[selectedMotor].encoderCount + JOG_STEP_SIZE;
  setGoal(selectedMotor, GOAL_MOVE_TO);

  Serial.print("Jogging CW to: ");
  Serial.println(jointState[selectedMotor].targetPosition);
}

void moveSelectedMotorCCW() {
  if (selectedMotor < 0) {
    Serial.println("No motor selected");
    return;
  }

  if (!jointState[selectedMotor].hasFoundHome) {
    Serial.println("Must home first");
    return;
  }

  // Check if already at or near CCW limit
  if (isCalibrated(selectedMotor)) {
    long nextPosition = jointState[selectedMotor].encoderCount - JOG_STEP_SIZE;
    if (nextPosition <= jointState[selectedMotor].maxEncoderStepsFromHomeCCW) {
      Serial.println("Cannot jog: at CCW limit");
      return;
    }
  }

  // Set goal to move to current position - jog step
  jointState[selectedMotor].targetPosition =
    jointState[selectedMotor].encoderCount - JOG_STEP_SIZE;
  setGoal(selectedMotor, GOAL_MOVE_TO);

  Serial.print("Jogging CCW to: ");
  Serial.println(jointState[selectedMotor].targetPosition);
}
