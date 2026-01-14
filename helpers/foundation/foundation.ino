/*
  SCORBOT motor controller


  */

#include <Arduino.h>

#include "/Users/steveturbek/Documents/scorbot_controller/scorbot_controller/scorbot.h"

/*
Home is 0
CW is positive steps
CCW is negative steps
for xample Base CCW to home: -7338 CW to home:5734

CW closes gripper "Righty-tighty"
CW for wrist roll is normal human wrist direction

*/

// Goal-based states
enum JointGoal {
  GOAL_IDLE,                 // No active goal, motor at rest
  GOAL_FIND_HOME,            // Find home switch and set position to 0
  GOAL_CALIBRATE_RANGE_CW,   // Find CW limits from home
  GOAL_CALIBRATE_RANGE_CCW,  // Find CCW limits from home
  GOAL_MOVE_TO,              // Move to target encoder position
  GOAL_FAULT                 // Error condition, requires reset
};

// Goal names for debugging
const char* MOTOR_GOAL_NAMES[] = {"IDLE", "FIND_HOME", "CALIBRATE_RANGE", "MOVE_TO", "FAULT"};

struct ScorbotJointState {
  bool hasFoundHome;  // Whether home switch has been located

  // Motor control
  int motorSpeed;  // value (-99 for CCW speed, 0 for stop and 99 for CW)
  // will be interpreted using the min numbers for the individual motor

  JointGoal currentGoal;        // Current goal being pursued
  unsigned long goalStartTime;  // When current goal started

  long targetEncoderCountToMoveTo;           // Target encoder count (for GOAL_MOVE_TO)
  long maxEncoderStepsFromHomeCW;            // CW limit from home.   (0 = uncalibrated)
  long maxEncoderStepsFromHomeCCW;           // CCW limit from home (0 = uncalibrated)
  unsigned long lastHomeSwitchDebounceTime;  // For home switch debouncing

  // Encoder tracking
  long encoderCount;      // Current encoder position
  long lastEncoderCount;  // Previous count for stall detection
  int lastEncoded;        // Last 2-bit encoder state (for quadrature)

  // Timing and stall detection
  unsigned long lastEncoderCheckTime;  // For stall detection interval
  int stallCounter;                    // Consecutive stall checks
  int totalStallsThisGoal;             // Total stalls since goal started (for fault detection)
};

// Array of states, one per motor (initialize with zeros)
ScorbotJointState jointState[ScorbotJointIndex_COUNT];

// ============================================================================
// DEBUG HELPERS
// ============================================================================

void printJointState(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  ScorbotJointState* s = &jointState[ScorbotJointIndex];

  Serial.println("==== Joint State ====");
  Serial.print("Motor: ");
  Serial.println(SCORBOT_REF[ScorbotJointIndex].name);
  Serial.print("hasFoundHome: ");
  Serial.println(s->hasFoundHome);
  Serial.print("motorSpeed: ");
  Serial.println(s->motorSpeed);
  Serial.print("currentGoal: ");
  Serial.println(MOTOR_GOAL_NAMES[s->currentGoal]);
  Serial.print("goalStartTime: ");
  Serial.println(s->goalStartTime);
  Serial.print("targetEncoderCountToMoveTo: ");
  Serial.println(s->targetEncoderCountToMoveTo);
  Serial.print("maxEncoderStepsFromHomeCW: ");
  Serial.println(s->maxEncoderStepsFromHomeCW);
  Serial.print("maxEncoderStepsFromHomeCCW: ");
  Serial.println(s->maxEncoderStepsFromHomeCCW);
  Serial.print("lastHomeSwitchDebounceTime: ");
  Serial.println(s->lastHomeSwitchDebounceTime);
  Serial.print("encoderCount: ");
  Serial.println(s->encoderCount);
  Serial.print("lastEncoderCount: ");
  Serial.println(s->lastEncoderCount);
  Serial.print("lastEncoded: ");
  Serial.println(s->lastEncoded);
  Serial.print("lastEncoderCheckTime: ");
  Serial.println(s->lastEncoderCheckTime);
  Serial.print("stallCounter: ");
  Serial.println(s->stallCounter);
  Serial.print("totalStallsThisGoal: ");
  Serial.println(s->totalStallsThisGoal);
  Serial.println("====================");
}

// ============================================================================
// STATE INITIALIZATION
// ============================================================================

inline void initializeAllJointStates() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    jointState[i].hasFoundHome = false;

    jointState[i].motorSpeed = 0;

    jointState[i].currentGoal = GOAL_IDLE;
    jointState[i].goalStartTime = 0;

    jointState[i].targetEncoderCountToMoveTo = 0;
    jointState[i].maxEncoderStepsFromHomeCW = 0;   // 0 = uncalibrated
    jointState[i].maxEncoderStepsFromHomeCCW = 0;  // 0 = uncalibrated
    jointState[i].lastHomeSwitchDebounceTime = 0;

    jointState[i].encoderCount = 0;
    jointState[i].lastEncoderCount = 0;
    jointState[i].lastEncoded = 0;

    jointState[i].lastEncoderCheckTime = 0;
    jointState[i].stallCounter = 0;
    jointState[i].totalStallsThisGoal = 0;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(500);  // Brief delay to allow Serial to initialize

  Serial.println("\n SCORBOT START ===========================");

  // Initialize hardware
  setupAllMotorPins();
  initializeAllJointStates();

  // Safety: stop all motors
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    stopMotor(i);
  }

  // Initialize encoder check time
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    jointState[i].lastEncoderCheckTime = millis();
  }

  // start goal base motor find home
  // setGoal(0, GOAL_FIND_HOME);
  setGoal(1, GOAL_FIND_HOME);
  // setGoal(2, GOAL_FIND_HOME);
  // setGoal(3, GOAL_FIND_HOME);
  // setGoal(4, GOAL_FIND_HOME);
  // setGoal(5, GOAL_FIND_HOME);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  updateAllEncoders();  // CRITICAL: Update all encoders every loop

  checkAllStalls();

  checkAllHomeSwitches();

  // Update all active goals
  doAllGoals();

  delay(1);
}

// ============================================================================
// ============================================================================
// ============================================================================

// ============================================================================
// GOAL UPDATE DISPATCHER

// Set goal for specific motor
inline void setGoal(int ScorbotJointIndex, JointGoal goal) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  // Reset goal-specific flags
  jointState[ScorbotJointIndex].goalStartTime = millis();
  resetStallDetection(ScorbotJointIndex);
  jointState[ScorbotJointIndex].totalStallsThisGoal = 0;  // Reset stall count for new goal

  switch (goal) {
    case GOAL_FIND_HOME:
      jointState[ScorbotJointIndex].hasFoundHome = false;
      setMotor(ScorbotJointIndex,
               99);  // % of effective power for that motor, positive is clockwise
      break;

      // case GOAL_GO_HOME:
      // doGoalGoHome(ScorbotJointIndex);
      //   break;

      // case GOAL_CALIBRATE_RANGE_CCW:
      //   doGoalCalibrateRange_CCW(ScorbotJointIndex);
      //   break;

      // case GOAL_MOVE_TO:
      //   doGoalMoveTo(ScorbotJointIndex);
      //   break;

    case GOAL_IDLE:
      stopMotor(
          ScorbotJointIndex);  // stop motor if goal set to idle. double safety from previous bug
      break;
    case GOAL_FAULT:
      // Nothing to do
      break;
  }
  // save the Goal
  jointState[ScorbotJointIndex].currentGoal = goal;
}

// ------------------------------------------------------------------------
// Update all active goals
void doAllGoals() {
  for (int ScorbotJointIndex = 0; ScorbotJointIndex < ScorbotJointIndex_COUNT;
       ScorbotJointIndex++) {
    if (jointState[ScorbotJointIndex].currentGoal != GOAL_IDLE &&
        jointState[ScorbotJointIndex].currentGoal != GOAL_FAULT) {
      doGoal(ScorbotJointIndex);
    }
  }
}

void doGoal(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  JointGoal goal = jointState[ScorbotJointIndex].currentGoal;

  // Timeouts
  const unsigned long GOAL_TIMEOUT_MS = 100000;  // Max time for any goal

  // Timeout check
  if (goal != GOAL_IDLE && goal != GOAL_FAULT) {
    if (millis() - jointState[ScorbotJointIndex].goalStartTime > GOAL_TIMEOUT_MS) {
      Serial.print("TIMEOUT: ");
      Serial.println(SCORBOT_REF[ScorbotJointIndex].name);
      stopMotor(ScorbotJointIndex);
      setGoal(ScorbotJointIndex, GOAL_FAULT);
      return;
    }
  }

  switch (goal) {
    case GOAL_FIND_HOME:
      doGoalFindHome(ScorbotJointIndex);
      break;

      // case GOAL_GO_HOME:
      // doGoalGoHome(ScorbotJointIndex);
      //   break;

      // case GOAL_CALIBRATE_RANGE_CCW:
      //   doGoalCalibrateRange_CCW(ScorbotJointIndex);
      //   break;

      // case GOAL_MOVE_TO:
      //   doGoalMoveTo(ScorbotJointIndex);
      //   break;

    case GOAL_IDLE:
    case GOAL_FAULT:
      // Nothing to do
      break;
  }
}

// ------------------------------------------------------------------------
void doGoalFindHome(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  // Allow motor startup time before checking motorSpeed (avoid false error on first loop)
  unsigned long timeSinceGoalStart = millis() - jointState[ScorbotJointIndex].goalStartTime;

  if (jointState[ScorbotJointIndex].currentGoal != GOAL_FIND_HOME ||
      (jointState[ScorbotJointIndex].motorSpeed == 0 && timeSinceGoalStart > 50)) {
    Serial.println("ERROR in doGoalFindHome(); currentGoal or motorSpeed ");
    setGoal(ScorbotJointIndex, GOAL_FAULT);
    return;
  }

  // Check if we JUST crossed a home edge recently
  unsigned long MillisSinceProgramStart = millis();
  unsigned long timeSinceLastEdge =
      MillisSinceProgramStart - jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime;

  // Success conditions: we found home AND it was recent (within last 100ms)
  if (jointState[ScorbotJointIndex].hasFoundHome && timeSinceLastEdge < 100) {
    // We just crossed home edge! Encoder is now at 0

    stopMotor(ScorbotJointIndex);
    setGoal(ScorbotJointIndex, GOAL_IDLE);

    // printJointState(ScorbotJointIndex);  // debug function

    Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
    Serial.println(": Home found!");
    return;
  }

  // Otherwise, keep searching
  // Motor is already moving (started in setGoal)
  // Stall detection will reverse direction if needed
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// Reset stall detection state - call when starting motor or changing direction
inline void resetStallDetection(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  jointState[ScorbotJointIndex].stallCounter = 0;
  jointState[ScorbotJointIndex].lastEncoderCount = jointState[ScorbotJointIndex].encoderCount;
  jointState[ScorbotJointIndex].lastEncoderCheckTime = millis();
}

inline void setMotor(int ScorbotJointIndex, int speed) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;
  if (speed < -99 || speed > 99)
    return;

  // motor has different minimum values depending on direction, from weight of arm
  int motor_min = 0;
  int pwmValue = 0;

  if (speed > 0) {  // clockwise
    motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CW;
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, HIGH);
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);
  } else if (speed < 0) {  // counter clockwise
    motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CCW;
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, HIGH);
  } else {  // stop
    stopMotor(ScorbotJointIndex);
    return;
  }

  jointState[ScorbotJointIndex].motorSpeed = speed;

  // Remap speed 1-99 to motor_min-255
  pwmValue = map(abs(speed), 0, 99, motor_min, 255);

  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, pwmValue);

  Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
  Serial.print(" motor PWM set to ");

  if (speed > 0)
    Serial.print("+");
  else
    Serial.print("-");

  Serial.println(pwmValue);
}

// ------------------------------------------------------------------------
// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, 0);
  jointState[ScorbotJointIndex].motorSpeed = 0;
}

// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================

// Update encoder for specific motor (polled quadrature decoding)
inline void updateEncoder(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  // Read current state of both encoder pins
  int MSB = digitalRead(SCORBOT_REF[ScorbotJointIndex].encoder_p0_pin);
  int LSB = digitalRead(SCORBOT_REF[ScorbotJointIndex].encoder_p1_pin);

  // Combine pins into 2-bit value (00, 01, 10, or 11)
  int encoded = (MSB << 1) | LSB;

  // Create 4-bit value from previous state and current state for transition lookup
  int sum = (jointState[ScorbotJointIndex].lastEncoded << 2) | encoded;

  // Quadrature decoding: check transition patterns to determine direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    jointState[ScorbotJointIndex].encoderCount++;  // Clockwise rotation
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    jointState[ScorbotJointIndex].encoderCount--;  // Counter-clockwise rotation
  }

  // Store current state for next comparison
  jointState[ScorbotJointIndex].lastEncoded = encoded;
}

// Update all encoders
inline void updateAllEncoders() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    updateEncoder(i);
  }
}

// ------------------------------------------------------------------------
// STALL

inline bool checkStall(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return false;
  if (jointState[ScorbotJointIndex].motorSpeed == 0)
    return false;

  const unsigned long STALL_CHECK_INTERVAL_MS = 50;  // how often to check

  unsigned long MillisSinceProgramStart = millis();

  if (MillisSinceProgramStart - jointState[ScorbotJointIndex].lastEncoderCheckTime <
      STALL_CHECK_INTERVAL_MS) {
    return false;
  }  // not enough time to determine stall yet

  // Different stall thresholds for problematic joints (elbow and wrist_pitch)
  unsigned long STALL_THRESHOLD_MS;
  int STALL_MIN_ENCODER_CHANGE;

  if (ScorbotJointIndex == MOTOR_ELBOW || ScorbotJointIndex == MOTOR_WRIST_PITCH) {
    // Less sensitive for elbow and wrist_pitch - need longer time to declare stall
    STALL_THRESHOLD_MS = 400;      // 200ms instead of 100ms (2x more time)
    STALL_MIN_ENCODER_CHANGE = 1;  // Only need 1 encoder step (instead of 2)
  } else {
    // Default for other motors
    STALL_THRESHOLD_MS = 100;      // Time without motion = stall
    STALL_MIN_ENCODER_CHANGE = 2;  // Min counts in interval
  }

  long currentCount = jointState[ScorbotJointIndex].encoderCount;
  long encoderChange = abs(currentCount - jointState[ScorbotJointIndex].lastEncoderCount);

  // Check if motor isn't moving enough (potential stall)
  if (encoderChange < STALL_MIN_ENCODER_CHANGE) {
    jointState[ScorbotJointIndex].stallCounter++;  // Increment consecutive stall detections

    // If stalled for long enough, report stall
    if (jointState[ScorbotJointIndex].stallCounter * STALL_CHECK_INTERVAL_MS >=
        STALL_THRESHOLD_MS) {
      jointState[ScorbotJointIndex].stallCounter = 0;  // Reset for next stall detection
      return true;                                     // STALL DETECTED
    }
  } else {
    jointState[ScorbotJointIndex].stallCounter = 0;  // Motor moving, reset counter
  }

  jointState[ScorbotJointIndex].lastEncoderCount = currentCount;
  jointState[ScorbotJointIndex].lastEncoderCheckTime = MillisSinceProgramStart;

  return false;
}
// ------------------------------------------------------------------------
inline void checkAllStalls() {
  for (int ScorbotJointIndex = 0; ScorbotJointIndex < ScorbotJointIndex_COUNT;
       ScorbotJointIndex++) {
    if (checkStall(ScorbotJointIndex)) {
      if (ScorbotJointIndex == 5 && jointState[ScorbotJointIndex].motorSpeed > 0 &&
          jointState[ScorbotJointIndex].currentGoal ==
              GOAL_FIND_HOME) {  // gripper, closing is actually a home position

        stopMotor(ScorbotJointIndex);
        setGoal(ScorbotJointIndex, GOAL_IDLE);

        jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime = millis();
        jointState[ScorbotJointIndex].encoderCount = 0;  // Set CW edge as zero
        jointState[ScorbotJointIndex].hasFoundHome = true;
        Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
        Serial.println(": Home found!");
        return;
      }

      // Increment total stalls for this goal
      jointState[ScorbotJointIndex].totalStallsThisGoal++;

      Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
      Serial.print(": STALL #");
      Serial.println(jointState[ScorbotJointIndex].totalStallsThisGoal);

      // Check if we've stalled too many times - go to FAULT state
      const int MAX_STALLS_BEFORE_FAULT = 2;
      if (jointState[ScorbotJointIndex].totalStallsThisGoal >= MAX_STALLS_BEFORE_FAULT) {
        Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
        Serial.print(": Too many stalls (");
        Serial.print(jointState[ScorbotJointIndex].totalStallsThisGoal);
        Serial.println("), entering FAULT state");

        stopMotor(ScorbotJointIndex);
        setGoal(ScorbotJointIndex, GOAL_FAULT);
        return;
      }

      // deal with the stall with business logic
      if (jointState[ScorbotJointIndex].currentGoal == GOAL_FIND_HOME) {
        if (jointState[ScorbotJointIndex].motorSpeed > 0) {
          // stalled when moving clockwise, we don't know which side, if either, it is stalled at,
          // so assume at end of CW rotation
          Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
          Serial.println(": stalled going clockwise, reversing");

          int reversedSpeed = jointState[ScorbotJointIndex].motorSpeed * -1;
          // stopMotor(ScorbotJointIndex);
          // delay(100);
          setMotor(ScorbotJointIndex, reversedSpeed);
          resetStallDetection(ScorbotJointIndex);

        } else if (jointState[ScorbotJointIndex].motorSpeed < 0) {
          // stalled when moving clockwise, we don't know which side, if either, it is stalled at,
          // so assume at end of CW rotation

          Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
          Serial.println(": stalled going counter clockwise, reversing");

          int reversedSpeed = jointState[ScorbotJointIndex].motorSpeed * -1;
          // stopMotor(ScorbotJointIndex);
          // delay(100);
          setMotor(ScorbotJointIndex, reversedSpeed);
          resetStallDetection(ScorbotJointIndex);
        } else {
          // motor is idle, something is weird, set goal to IDLE
          jointState[ScorbotJointIndex].currentGoal = GOAL_IDLE;
        }
      }
    }
  }
  return;
}
// ============================================================================
// Home Switches
// ============================================================================

// Check if home switch pressed
inline bool isHomeSwitchPressed(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return false;

  // Simply return the current state - switch is active LOW
  return (digitalRead(SCORBOT_REF[ScorbotJointIndex].home_switch_pin) == LOW);
}

// ------------------------------------------------------------------------
inline bool checkIfHomeSwitchEdge(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return false;

  unsigned long MillisSinceProgramStart = millis();
  const unsigned long SWITCH_DEBOUNCE_MS = 10;  // Switch debounce time

  // jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime is last time the switch was pressed
  if (MillisSinceProgramStart - jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime <
      SWITCH_DEBOUNCE_MS)
    return false;  // ignore to debounce

  // Simply return the current state - switch is active LOW
  if (isHomeSwitchPressed(ScorbotJointIndex)) {
    // we are really on the switch

    if (jointState[ScorbotJointIndex].motorSpeed > 0) {
      Serial.println("hit switch going clockwise, reset home position");
      // if going clockwise, reset home position
      jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime = MillisSinceProgramStart;
      jointState[ScorbotJointIndex].encoderCount = 0;  // Set CW edge as zero
      jointState[ScorbotJointIndex].hasFoundHome = true;
      return true;

    } else {
      jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime =
          MillisSinceProgramStart;  // hit switch going CCW,
      return false;  // if stopped or going counter clockwise, ignore till we fall off the switch
    }
  } else {  // switch is low

    if (jointState[ScorbotJointIndex].motorSpeed < 0 &&
        jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime != 0 &&
        MillisSinceProgramStart - jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime >
            SWITCH_DEBOUNCE_MS * 10) {
      // if going counter clockwise, reset home position
      // just rotated off the switch

      Serial.println("off switch going counter clockwise, reset home position");
      // future task: validate how many steps difference from CW rising edge to CCW falling edge

      jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime = MillisSinceProgramStart;
      jointState[ScorbotJointIndex].encoderCount = 0;  // Set CW edge as zero
      jointState[ScorbotJointIndex].hasFoundHome = true;
      return true;
    }
  }

  return false;
}

inline void checkAllHomeSwitches() {
  for (int ScorbotJointIndex = 0; ScorbotJointIndex < ScorbotJointIndex_COUNT;
       ScorbotJointIndex++) {
    if (checkIfHomeSwitchEdge(ScorbotJointIndex)) {
      // deal with the stall with business logic
    }
  }
  return;
}