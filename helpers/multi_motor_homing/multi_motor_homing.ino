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
  - 'g' = Go to home position for selected motor (must be homed first)
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
  Serial.println("  g       = Go to home (selected motor)");
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
  doAllGoals();

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

      case 'g':
      case 'G':
        if (selectedMotor >= 0) {
          if (jointState[selectedMotor].hasFoundHome) {
            Serial.print("Returning to home: ");
            Serial.println(SCORBOT_REF[selectedMotor].name);
            jointState[selectedMotor].targetPosition = 0;
            setGoal(selectedMotor, GOAL_MOVE_TO);
          } else {
            Serial.println("Must find home first (press 'f')");
          }
        } else {
          Serial.println("No motor selected");
        }
        break;

      case '+':
      case '=':
        jogSelectedMotorCW();
        break;

      case '-':
      case '_':
        jogSelectedMotorCCW();
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
void doAllGoals() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    if (jointState[i].currentGoal != GOAL_IDLE &&
        jointState[i].currentGoal != GOAL_FAULT) {
      doGoal(i);
    }
  }
}

void doGoal(int motorIndex) {
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
      doGoalFindHome(motorIndex);
      break;

    case GOAL_CALIBRATE_RANGE:
      doGoalCalibrateRange(motorIndex);
      break;

    case GOAL_MOVE_TO:
      doGoalMoveTo(motorIndex);
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

void doGoalFindHome(int motorIndex) {
  // If we find the switch and we've searched BOTH directions (moved CW off switch, then back CCW), this is the CW edge
  if (isHomeSwitchPressed(motorIndex) &&
      jointState[motorIndex].searchedCW &&
      jointState[motorIndex].searchedCCW) {
    stopMotor(motorIndex);
    jointState[motorIndex].encoderCount = 0;  // Set CW edge as zero
    jointState[motorIndex].hasFoundHome = true;
    setGoal(motorIndex, GOAL_IDLE);
    Serial.print(SCORBOT_REF[motorIndex].name);
    Serial.println(": Home found at CCW side of switch.");
    return;
  }

  // If switch is NOT pressed and we haven't moved yet, start by checking if we're already on the switch
  if (!jointState[motorIndex].motorActive) {
    // Check if we're starting on the switch - if so, move CW to get off it first
    if (isHomeSwitchPressed(motorIndex)) {
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Starting on switch, moving CW to clear it...");
      moveMotorCW(motorIndex, 255);
      jointState[motorIndex].searchedCW = true;
      return;
    } else {
      // Not on switch - start search by moving CCW to find it
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Starting CCW search...");
      moveMotorCCW(motorIndex, 255);
      jointState[motorIndex].searchedCCW = true;
      return;
    }
  }

  // If we're moving CW (to clear the switch) and switch is no longer pressed, reverse direction
  if (jointState[motorIndex].searchedCW && !jointState[motorIndex].searchedCCW &&
      !isHomeSwitchPressed(motorIndex)) {
    stopMotor(motorIndex);
    delay(50);
    Serial.print(SCORBOT_REF[motorIndex].name);
    Serial.println(": Switch cleared, reversing to find CW edge...");
    // Reset stall detection for new movement
    jointState[motorIndex].stallCounter = 0;
    jointState[motorIndex].lastEncoderCount = jointState[motorIndex].encoderCount;
    jointState[motorIndex].lastEncoderCheckTime = millis();
    moveMotorCCW(motorIndex, 255);
    jointState[motorIndex].searchedCCW = true;
    return;
  }

  // Check for stall (hit limit) - only applies during initial search, not during controlled approach
  if (checkStall(motorIndex)) {
    stopMotor(motorIndex);
    delay(100);

    // Check if we've tried both directions during initial search
    if (jointState[motorIndex].searchedCCW && !jointState[motorIndex].searchedCW) {
      // CCW stalled without finding switch - try CW
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": CCW stalled, trying CW");
      jointState[motorIndex].searchedCW = true;
      moveMotorCW(motorIndex, 255);
    } else if (jointState[motorIndex].searchedCW && !jointState[motorIndex].searchedCCW) {
      // CW stalled - try CCW
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": CW stalled, trying CCW");
      jointState[motorIndex].searchedCCW = true;
      moveMotorCCW(motorIndex, 255);
    } else if (jointState[motorIndex].searchedCW && jointState[motorIndex].searchedCCW) {
      // Tried both directions and stalled - could be in approach phase OR couldn't find switch
      // Only fault if we never found the switch
      if (!jointState[motorIndex].hasFoundHome) {
        Serial.print(SCORBOT_REF[motorIndex].name);
        Serial.println(": FAULT - home switch not found in either direction");
        setGoal(motorIndex, GOAL_FAULT);
      } else {
        // We found home before, so this is a stall during approach
        Serial.print(SCORBOT_REF[motorIndex].name);
        Serial.println(": FAULT - stalled during approach to switch");
        setGoal(motorIndex, GOAL_FAULT);
      }
    }
  }
}

// ============================================================================
// GOAL: CALIBRATE_RANGE
// ============================================================================

void doGoalCalibrateRange(int motorIndex) {
  // Must have home first
  if (!jointState[motorIndex].hasFoundHome) {
    Serial.println("Error: Need home before calibrating");
    setGoal(motorIndex, GOAL_FAULT);
    return;
  }

  // Phase 1: Find CCW limit (move away from home first)
  if (jointState[motorIndex].calibrationPhase == 1) {
    if (!jointState[motorIndex].motorActive) {
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Finding CCW limit...");
      moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
    }

    if (checkStall(motorIndex)) {
      jointState[motorIndex].maxEncoderStepsFromHomeCCW =
        jointState[motorIndex].encoderCount;
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.print(": CCW limit = ");
      Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
      Serial.print(" (");
      Serial.print(abs(jointState[motorIndex].maxEncoderStepsFromHomeCCW));
      Serial.println(" steps from home)");
      stopMotor(motorIndex);
      delay(100);
      jointState[motorIndex].calibrationPhase = 2;  // Move to phase 2
    }
    return;
  }

  // Phase 2: Find CW limit (move back through home toward CW)
  // IMPORTANT: Ignore home switch during this phase - we're passing through it
  if (jointState[motorIndex].calibrationPhase == 2) {
    if (!jointState[motorIndex].motorActive) {
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.println(": Finding CW limit...");
      moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
    }

    if (checkStall(motorIndex)) {
      jointState[motorIndex].maxEncoderStepsFromHomeCW =
        jointState[motorIndex].encoderCount;
      Serial.print(SCORBOT_REF[motorIndex].name);
      Serial.print(": CW limit = ");
      Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCW);
      Serial.print(" (");
      Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCW);
      Serial.println(" steps from home)");
      stopMotor(motorIndex);
      delay(100);
      jointState[motorIndex].calibrationPhase = 3;  // Move to phase 3
    }
    return;
  }

  // Phase 3: Return to home (approach from CW side for consistency)
  if (jointState[motorIndex].calibrationPhase == 3) {
    if (abs(jointState[motorIndex].encoderCount) > HOME_POSITION_TOLERANCE) {
      if (!jointState[motorIndex].motorActive) {
        // Move toward home from whichever side we're on
        if (jointState[motorIndex].encoderCount > 0) {
          // We're on CW side, move CCW toward home
          moveMotorCCW(motorIndex, HOMING_SEARCH_SPEED);
        } else {
          // We're on CCW side, move CW toward home
          moveMotorCW(motorIndex, HOMING_SEARCH_SPEED);
        }
      }

      // When we hit the switch, set position to 0 (CW edge)
      if (isHomeSwitchPressed(motorIndex)) {
        stopMotor(motorIndex);
        jointState[motorIndex].encoderCount = 0;
        setGoal(motorIndex, GOAL_IDLE);
        Serial.print(SCORBOT_REF[motorIndex].name);
        Serial.print(": Calibration complete. CCW to home: ");
        Serial.print(jointState[motorIndex].maxEncoderStepsFromHomeCCW);
        Serial.print(" CW to home:");
        Serial.println(jointState[motorIndex].maxEncoderStepsFromHomeCW);
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
}

// ============================================================================
// GOAL: MOVE_TO (placeholder for future)
// ============================================================================

void doGoalMoveTo(int motorIndex) {
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


void jogSelectedMotorCW() {
  if (selectedMotor < 0) {
    Serial.println("No motor selected");
    return;
  }

  // if (!jointState[selectedMotor].hasFoundHome) {
  //   Serial.println("Must home first");
  //   return;
  // }

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

void jogSelectedMotorCCW() {
  if (selectedMotor < 0) {
    Serial.println("No motor selected");
    return;
  }

  // if (!jointState[selectedMotor].hasFoundHome) {
  //   Serial.println("Must home first");
  //   return;
  // }

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
