/*
  SCORBOT motor controller


  */

// Goal-based states
enum MotorGoal {
  GOAL_IDLE,             // No active goal, motor at rest
  GOAL_FIND_HOME,        // Find home switch and set position to 0
  GOAL_CALIBRATE_RANGE,  // Find CW and CCW limits from home
  GOAL_MOVE_TO,          // Move to target encoder position
  GOAL_FAULT             // Error condition, requires reset
};

// Goal names for debugging
const char* MOTOR_GOAL_NAMES[] = {"IDLE", "FIND_HOME", "CALIBRATE_RANGE", "MOVE_TO", "FAULT"};

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

struct ScorbotJointState {
  // Encoder tracking
  long encoderCount;      // Current encoder position
  long lastEncoderCount;  // Previous count for stall detection
  int lastEncoded;        // Last 2-bit encoder state (for quadrature)

  // Motor control
  int motorPWM;  // Current PWM value (-255 for CCW speed, 0 for stop and 255 for CW)

  // Goal-based state
  MotorGoal currentGoal;  // Current goal being pursued
  long targetPosition;    // Target encoder count (for GOAL_MOVE_TO)

  // Calibration data (0 = uncalibrated)
  long maxEncoderStepsFromHomeCW;   // CW limit from home
  long maxEncoderStepsFromHomeCCW;  // CCW limit from home

  // Timing and stall detection
  unsigned long lastEncoderCheckTime;    // For stall detection interval
  unsigned long lastSwitchDebounceTime;  // For switch debouncing
  unsigned long goalStartTime;           // When current goal started
  int stallCounter;                      // Consecutive stall checks
};

// Array of states, one per motor (initialize with zeros)
ScorbotJointState jointState[ScorbotJointIndex_COUNT];

// ============================================================================
// STATE INITIALIZATION
// ============================================================================

inline void initializeAllJointStates() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    jointState[i].encoderCount = 0;
    jointState[i].lastEncoderCount = 0;
    jointState[i].lastEncoded = 0;

    jointState[i].motorPWM = 0;

    jointState[i].currentGoal = GOAL_IDLE;
    jointState[i].targetPosition = 0;

    jointState[i].maxEncoderStepsFromHomeCW = 0;   // 0 = uncalibrated
    jointState[i].maxEncoderStepsFromHomeCCW = 0;  // 0 = uncalibrated

    jointState[i].lastEncoderCheckTime = 0;
    jointState[i].lastSwitchDebounceTime = 0;
    jointState[i].goalStartTime = 0;
    jointState[i].stallCounter = 0;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println("\n===========SCORBOT START================");

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
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  updateAllEncoders();  // CRITICAL: Update all encoders every loop

  checkAllStalls();

  // Update all active goals
  doAllGoals();

  delay(1);
}

// ============================================================================
// ============================================================================
// ============================================================================

// ============================================================================
// GOAL UPDATE DISPATCHER
// ============================================================================

// Update all active goals
void doAllGoals() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    if (jointState[i].currentGoal != GOAL_IDLE && jointState[i].currentGoal != GOAL_FAULT) {
      doGoal(i);
    }
  }
}

void doGoal(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT)
    return;

  MotorGoal goal = jointState[motorIndex].currentGoal;

  // Timeouts
  const unsigned long GOAL_TIMEOUT_MS = 100000;  // Max time for any goal

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
      //   break;

      // case GOAL_CALIBRATE_RANGE:
      //   doGoalCalibrateRange(motorIndex);
      //   break;

      // case GOAL_MOVE_TO:
      //   doGoalMoveTo(motorIndex);
      //   break;

    case GOAL_IDLE:
    case GOAL_FAULT:
      // Nothing to do
      break;
  }
}

// Set goal for specific motor
inline void setGoal(int motorIndex, MotorGoal goal) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT)
    return;

  // Reset goal-specific flags
  jointState[motorIndex].stallCounter = 0;
  jointState[motorIndex].goalStartTime = millis();

  // stop motor if goal set to idle. double safety from previous bug
  if (goal == GOAL_IDLE) {
    stopMotor(motorIndex);
  }

  jointState[motorIndex].currentGoal = goal;
}

// ============================================================================
// GOAL: FIND_HOME
// ============================================================================

void doGoalFindHome(int motorIndex) {}

// ============================================================================
// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT)
    return;

  digitalWrite(SCORBOT_REF[motorIndex].CCW_pin, LOW);
  digitalWrite(SCORBOT_REF[motorIndex].CW_pin, LOW);
  analogWrite(SCORBOT_REF[motorIndex].pwm_pin, 0);
  jointState[motorIndex].motorPWM = 0;
}

// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================

// Update encoder for specific motor (polled quadrature decoding)
inline void updateEncoder(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT)
    return;

  int MSB = digitalRead(SCORBOT_REF[motorIndex].encoder_p0_pin);
  int LSB = digitalRead(SCORBOT_REF[motorIndex].encoder_p1_pin);

  int encoded = (MSB << 1) | LSB;
  int sum = (jointState[motorIndex].lastEncoded << 2) | encoded;

  // Quadrature decoding
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    jointState[motorIndex].encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    jointState[motorIndex].encoderCount--;
  }

  jointState[motorIndex].lastEncoded = encoded;
}

// Update all encoders
inline void updateAllEncoders() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    updateEncoder(i);
  }
}

// ============================================================================
// STALL
// ============================================================================

inline bool checkStall(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT)
    return false;
  if (jointState[motorIndex].motorPWM == 0)
    return false;

  const unsigned long STALL_CHECK_INTERVAL_MS = 50;  // how often to check

  unsigned long currentTime = millis();

  if (currentTime - jointState[motorIndex].lastEncoderCheckTime < STALL_CHECK_INTERVAL_MS) {
    return false;
  }  // not enough time to determine stall yet

  const unsigned long STALL_THRESHOLD_MS = 100;  // Time without motion = stall
  const int STALL_MIN_ENCODER_CHANGE = 2;        // Min counts in interval

  long currentCount = jointState[motorIndex].encoderCount;
  long encoderChange = abs(currentCount - jointState[motorIndex].lastEncoderCount);

  // Check if motor isn't moving enough (potential stall)
  if (encoderChange < STALL_MIN_ENCODER_CHANGE) {
    jointState[motorIndex].stallCounter++;  // Increment consecutive stall detections

    // If stalled for long enough, report stall
    if (jointState[motorIndex].stallCounter * STALL_CHECK_INTERVAL_MS >= STALL_THRESHOLD_MS) {
      jointState[motorIndex].stallCounter = 0;  // Reset for next stall detection
      return true;                              // STALL DETECTED
    }
  } else {
    jointState[motorIndex].stallCounter = 0;  // Motor moving, reset counter
  }

  jointState[motorIndex].lastEncoderCount = currentCount;
  jointState[motorIndex].lastEncoderCheckTime = currentTime;

  return false;
}

inline void checkAllStalls() {
  for (int motorIndex = 0; motorIndex < ScorbotJointIndex_COUNT; motorIndex++) {
    if (checkStall(motorIndex)) {
      if (jointState[motorIndex].motorPWM > 0) {
        Serial.print(SCORBOT_REF[motorIndex].name);
        Serial.println(": stalled going clockwise, reversing");
        jointState[motorIndex].motorPWM *= -1;
      }

      // stopMotor(motorIndex);
      // delay(100);
    }
  }
}