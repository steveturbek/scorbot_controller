/*
SCORBOT ER III - PIN DEFINITIONS

This file defines all Arduino Mega pin assignments for the Scorbot robot.
Include this file in any sketch to ensure consistent pin usage.

Based on DB-50 connector wiring and L298N motor driver configuration.


  Access motor pins through the `SCORBOT_REF[]` array:

  ```arduino
  // Move base motor clockwise at 50% speed
  digitalWrite(SCORBOT_REF[MOTOR_BASE].dir1, HIGH);
  digitalWrite(SCORBOT_REF[MOTOR_BASE].dir2, LOW);
  analogWrite(SCORBOT_REF[MOTOR_BASE].pwm, 128);

  // Check shoulder home switch
  bool atHome = (digitalRead(SCORBOT_REF[MOTOR_SHOULDER].home_switch) == LOW);

  // Read elbow encoder
  int p0 = digitalRead(SCORBOT_REF[MOTOR_ELBOW].encoder_p0);
  int p1 = digitalRead(SCORBOT_REF[MOTOR_ELBOW].encoder_p1);

  // Print motor name for debugging
  Serial.println(SCORBOT_REF[MOTOR_BASE].name);  // Prints "Base"

*/

#ifndef SCORBOT_PINS_H
#define SCORBOT_PINS_H

#include <string.h>  // For strcmp() function

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================
// Note: Serial1 uses pins 18 (TX) and 19 (RX) on Arduino Mega
// These are used for communication with external joystick controller
#define SERIAL_TX               18  // Reserved for Serial1
#define SERIAL_RX               19  // Reserved for Serial1


struct Scorbot_joints_reference {
  int CCW_pin;           // Direction pin (H-bridge IN1 or IN3)
  int CW_pin;              // Direction pin (H-bridge IN2 or IN4)
  int pwm_pin;           // PWM speed control pin (H-bridge ENA or ENB)
  int encoder_p0_pin;    // Encoder channel P0
  int encoder_p1_pin;    // Encoder channel P1
  int home_switch_pin;   // Home microswitch pin
  const char* name;  // Motor name for debugging
   int motor_min_CCW;     // minimum to make motor move
   int motor_min_CW;     // minimum to make motor move
};

// Motor axis indices (for array access)
enum ScorbotJointIndex {
  MOTOR_BASE = 0,
  MOTOR_SHOULDER,
  MOTOR_ELBOW,
  MOTOR_WRIST_PITCH,
  MOTOR_WRIST_ROLL,
  MOTOR_GRIPPER,
  ScorbotJointIndex_COUNT // this is the count of the enum, gets a value assigned automagically 
};

// Main Scorbot motor configuration array
// Usage: SCORBOT_REF[MOTOR_BASE].pwm_pin or SCORBOT_REF[0].pwm_pin
// Using array designators to prevent enum/array order mismatch
const Scorbot_joints_reference SCORBOT_REF[ScorbotJointIndex_COUNT] = {
  
// ============================================================================
//  BASE (310° rotation)
// L298N Board 1, Motor A (Output 1)
    [MOTOR_BASE] = {
    .CCW_pin = 22,
    .CW_pin = 23, 
    .pwm_pin = 7, 
    .encoder_p0_pin = 34,
    .encoder_p1_pin = 35,
    .home_switch_pin = 46,
    .name =  "base",
    .motor_min_CCW = 40, //0-255
    .motor_min_CW = 45 //0-255
  },




// ============================================================================
// SHOULDER (+130° / -35°)
// ============================================================================
// L298N Board 1, Motor B (Output 2)

  [MOTOR_SHOULDER] = {
    .CCW_pin = 24,
    .CW_pin = 25,
    .pwm_pin = 6,
    .encoder_p0_pin = 36,
    .encoder_p1_pin = 37,
    .home_switch_pin = 47,
    .name = "shoulder",
    .motor_min_CCW = 200, //0-255
    .motor_min_CW = 0 //0-255
  },




// ============================================================================
// ELBOW (±130°)
// ============================================================================
// L298N Board 2, Motor A (Output 1)

  [MOTOR_ELBOW] = {
    .CCW_pin = 27,
    .CW_pin = 26,
    .pwm_pin = 5,
    .encoder_p0_pin = 38,
    .encoder_p1_pin = 39,
    .home_switch_pin = 48,
    .name = "elbow",
    .motor_min_CCW = 250, //0-255
    .motor_min_CW = 100 //0-255
  },




// ============================================================================
// WRIST PITCH (±130°)
// ============================================================================
// L298N Board 2, Motor B (Output 2)
  [MOTOR_WRIST_PITCH] = {
    .CCW_pin = 29,
    .CW_pin = 28,
    .pwm_pin = 4,
    .encoder_p0_pin = 40,
    .encoder_p1_pin = 41,
    .home_switch_pin = 49,
    .name = "wrist_pitch",
    .motor_min_CCW = 200, //0-255
    .motor_min_CW = 200 //0-255
  },


// ============================================================================
// WRIST ROLL (unlimited rotation)
// ============================================================================
// L298N Board 3, Motor A (Output 1)
  [MOTOR_WRIST_ROLL] = {
    .CCW_pin = 30,
    .CW_pin = 31,
    .pwm_pin = 3,
    .encoder_p0_pin = 42,
    .encoder_p1_pin = 43,
    .home_switch_pin = 50,
    .name = "wrist_roll",
    .motor_min_CCW = 200, //0-255
    .motor_min_CW = 200 //0-255
  },


// ============================================================================
// GRIPPER
// ============================================================================
// L298N Board 3, Motor B (Output 2)
  [MOTOR_GRIPPER] = {
    .CCW_pin = 32,
    .CW_pin = 33,
    .pwm_pin = 2,
    .encoder_p0_pin = 44,
    .encoder_p1_pin = 45,
    .home_switch_pin = 51,
    .name = "gripper",
    .motor_min_CCW = 100, //0-255
    .motor_min_CW = 100 //0-255
  }
};

// ============================================================================
// ACTIVE STATE OF EACH SCORBOT JOINT
// ============================================================================


// Goal-based state management (replaces detailed state machine)
enum MotorGoal {
  GOAL_IDLE,              // No active goal, motor at rest
  GOAL_FIND_HOME,         // Find home switch and set position to 0
  GOAL_CALIBRATE_RANGE,   // Find CW and CCW limits from home
  GOAL_MOVE_TO,           // Move to target encoder position
  GOAL_FAULT              // Error condition, requires reset
};

// Goal names for debugging
const char* MOTOR_GOAL_NAMES[] = {
  "IDLE",
  "FIND_HOME",
  "CALIBRATE_RANGE",
  "MOVE_TO",
  "FAULT"
};

struct ScorbotJointState {
  // Encoder tracking
  long encoderCount;           // Current encoder position
  long lastEncoderCount;       // Previous count for stall detection
  int lastEncoded;             // Last 2-bit encoder state (for quadrature)

  // Motor control
  int motorPWM;                // Current PWM value (0-255)
  bool motorActive;            // Whether motor is currently powered

  // Goal-based state
  MotorGoal currentGoal;       // Current goal being pursued
  long targetPosition;         // Target encoder count (for GOAL_MOVE_TO)

  // Calibration data (0 = uncalibrated)
  long maxEncoderStepsFromHomeCW;   // CW limit from home
  long maxEncoderStepsFromHomeCCW;  // CCW limit from home

  // Progress flags (minimal, goal-specific)
  bool hasFoundHome;           // Whether home switch has been located
  bool searchedCCW;            // During GOAL_FIND_HOME: tried CCW direction
  bool searchedCW;             // During GOAL_FIND_HOME: tried CW direction
  int calibrationPhase;        // During GOAL_CALIBRATE_RANGE: 1=CCW limit, 2=CW limit, 3=return home

  // Timing and stall detection
  unsigned long lastEncoderCheckTime;     // For stall detection interval
  unsigned long lastSwitchDebounceTime;   // For switch debouncing
  unsigned long goalStartTime;            // When current goal started
  int stallCounter;                       // Consecutive stall checks
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
    jointState[i].motorActive = false;
    jointState[i].currentGoal = GOAL_IDLE;
    jointState[i].targetPosition = 0;
    jointState[i].maxEncoderStepsFromHomeCW = 0;    // 0 = uncalibrated
    jointState[i].maxEncoderStepsFromHomeCCW = 0;   // 0 = uncalibrated
    jointState[i].hasFoundHome = false;
    jointState[i].searchedCCW = false;
    jointState[i].searchedCW = false;
    jointState[i].calibrationPhase = 0;
    jointState[i].lastEncoderCheckTime = 0;
    jointState[i].lastSwitchDebounceTime = 0;
    jointState[i].goalStartTime = 0;
    jointState[i].stallCounter = 0;
  }
}



// ============================================================================
// HOMING AND CALIBRATION CONSTANTS
// ============================================================================

// Movement speeds
const int HOMING_SEARCH_SPEED = 200;      // PWM for initial search (0-255)
const int HOMING_APPROACH_SPEED = 100;    // PWM for final approach

// Timeouts
const unsigned long GOAL_TIMEOUT_MS = 100000;  // Max time for any goal

// Stall detection
const unsigned long STALL_CHECK_INTERVAL_MS = 50;  // Check interval
const unsigned long STALL_THRESHOLD_MS = 100;      // Time without motion = stall
const int STALL_MIN_ENCODER_CHANGE = 2;            // Min counts in interval

// Debouncing
const unsigned long SWITCH_DEBOUNCE_MS = 10;       // Switch debounce time

// Position tolerance
const int HOME_POSITION_TOLERANCE = 10;  // Encoder counts near 0 = "at home"

// Jog distance in encoder counts
const int JOG_STEP_SIZE = 100;


// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// Configure all pins for a single motor
// Usage: setupMotor(MOTOR_BASE);
inline void setupMotor(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT) return;

  pinMode(SCORBOT_REF[ScorbotJointIndex].CCW_pin, OUTPUT);
  pinMode(SCORBOT_REF[ScorbotJointIndex].CW_pin, OUTPUT);
  pinMode(SCORBOT_REF[ScorbotJointIndex].pwm_pin, OUTPUT);
  pinMode(SCORBOT_REF[ScorbotJointIndex].encoder_p0_pin, INPUT_PULLUP);
  pinMode(SCORBOT_REF[ScorbotJointIndex].encoder_p1_pin, INPUT_PULLUP);
  pinMode(SCORBOT_REF[ScorbotJointIndex].home_switch_pin, INPUT_PULLUP);
}
// ============================================================================
// Configure all 6 motors at once
// Usage: setupAllMotors();
inline void setupAllMotors() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    setupMotor(i);
  }
}
// ============================================================================
// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;

  digitalWrite(SCORBOT_REF[motorIndex].CCW_pin, LOW);
  digitalWrite(SCORBOT_REF[motorIndex].CW_pin, LOW);
  analogWrite(SCORBOT_REF[motorIndex].pwm_pin, 0);
  jointState[motorIndex].motorPWM = 0;
  jointState[motorIndex].motorActive = false;
}

// Move motor clockwise
inline void moveMotorCW(int motorIndex, int speed) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;
  if (speed < 0 || speed > 255) return;

  digitalWrite(SCORBOT_REF[motorIndex].CW_pin, HIGH);
  digitalWrite(SCORBOT_REF[motorIndex].CCW_pin, LOW);
  analogWrite(SCORBOT_REF[motorIndex].pwm_pin, speed);
  jointState[motorIndex].motorPWM = speed;
  jointState[motorIndex].motorActive = true;
}

// Move motor counter-clockwise
inline void moveMotorCCW(int motorIndex, int speed) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;
  if (speed < 0 || speed > 255) return;

  digitalWrite(SCORBOT_REF[motorIndex].CCW_pin, HIGH);
  digitalWrite(SCORBOT_REF[motorIndex].CW_pin, LOW);
  analogWrite(SCORBOT_REF[motorIndex].pwm_pin, speed);
  jointState[motorIndex].motorPWM = speed;
  jointState[motorIndex].motorActive = true;
}

// ============================================================================
// Move a motor
// Usage: moveMotor(MOTOR_BASE, 50, true);  // turn CW at speed 50% (range: 0-99)
inline void moveMotor(int ScorbotJointIndex, int speed, bool clockwise) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT) return;
  if (speed < 0 || speed > 255) return;

  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, clockwise ? HIGH : LOW);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, clockwise ? LOW : HIGH);


  int pwmValue;
  if (speed == 0) {
    pwmValue = 0;  // Stop motor
  } else {
// motor has different minimum values depending on direction
    int motor_min = clockwise? SCORBOT_REF[ScorbotJointIndex].motor_min_CW : SCORBOT_REF[ScorbotJointIndex].motor_min_CCW;

    // Remap speed 1-99 to motor_min-255
    pwmValue = map(speed, 1, 99, motor_min, 255);
    }

  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, pwmValue);
}


// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================

// Update encoder for specific motor (polled quadrature decoding)
inline void updateEncoder(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;

  int MSB = digitalRead(SCORBOT_REF[motorIndex].encoder_p0_pin);
  int LSB = digitalRead(SCORBOT_REF[motorIndex].encoder_p1_pin);

  int encoded = (MSB << 1) | LSB;
  int sum = (jointState[motorIndex].lastEncoded << 2) | encoded;

  // Quadrature decoding
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    jointState[motorIndex].encoderCount++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
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
// SAFETY AND STATUS FUNCTIONS
// ============================================================================

// Check if home switch pressed
inline bool isHomeSwitchPressed(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;

  // Simply return the current state - switch is active LOW
  return (digitalRead(SCORBOT_REF[motorIndex].home_switch_pin) == LOW);
}

// Check for stall condition
inline bool checkStall(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;
  if (!jointState[motorIndex].motorActive) return false;

  unsigned long currentTime = millis();

  if (currentTime - jointState[motorIndex].lastEncoderCheckTime >= STALL_CHECK_INTERVAL_MS) {
    long currentCount = jointState[motorIndex].encoderCount;
    long encoderChange = abs(currentCount - jointState[motorIndex].lastEncoderCount);

    if (encoderChange < STALL_MIN_ENCODER_CHANGE) {
      jointState[motorIndex].stallCounter++;
      if (jointState[motorIndex].stallCounter * STALL_CHECK_INTERVAL_MS >= STALL_THRESHOLD_MS) {
        jointState[motorIndex].stallCounter = 0;
        return true;
      }
    } else {
      jointState[motorIndex].stallCounter = 0;
    }

    jointState[motorIndex].lastEncoderCount = currentCount;
    jointState[motorIndex].lastEncoderCheckTime = currentTime;
  }

  return false;
}

// Check if motor is calibrated (both limits known)
inline bool isCalibrated(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;
  return (jointState[motorIndex].maxEncoderStepsFromHomeCW != 0 ||
          jointState[motorIndex].maxEncoderStepsFromHomeCCW != 0);
}

// Check if position is within safe range
inline bool isInSafeRange(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;
  if (!isCalibrated(motorIndex)) return true; // No limits to check yet

  long count = jointState[motorIndex].encoderCount;
  return (count >= jointState[motorIndex].maxEncoderStepsFromHomeCCW &&
          count <= jointState[motorIndex].maxEncoderStepsFromHomeCW);
}

// Check if near limits
inline bool isNearLimit(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;
  if (!isCalibrated(motorIndex)) return false;

  long count = jointState[motorIndex].encoderCount;
  const int margin = 100;
  return (count <= (jointState[motorIndex].maxEncoderStepsFromHomeCCW + margin) ||
          count >= (jointState[motorIndex].maxEncoderStepsFromHomeCW - margin));
}

// Check if at home position
inline bool isAtHome(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return false;
  return (abs(jointState[motorIndex].encoderCount) <= HOME_POSITION_TOLERANCE &&
          isHomeSwitchPressed(motorIndex));
}

// ============================================================================
// GOAL CONTROL FUNCTIONS
// ============================================================================

// Set goal for specific motor
inline void setGoal(int motorIndex, MotorGoal goal) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;

  // Reset goal-specific flags
  jointState[motorIndex].searchedCCW = false;
  jointState[motorIndex].searchedCW = false;
  jointState[motorIndex].stallCounter = 0;
  jointState[motorIndex].goalStartTime = millis();


//stop motor if goal set to idle. double safety from previous bug
  if (goal == GOAL_IDLE ) {
      stopMotor(motorIndex);
    }


  // Reset calibration phase when starting calibration
  if (goal == GOAL_CALIBRATE_RANGE) {
    jointState[motorIndex].calibrationPhase = 1;  // Start at phase 1 (CCW limit)
  }

  jointState[motorIndex].currentGoal = goal;
}

// Start homing for specific motor (find home + calibrate)
inline void startHoming(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= ScorbotJointIndex_COUNT) return;
  stopMotor(motorIndex);
  // Reset calibration data for fresh homing sequence
  jointState[motorIndex].maxEncoderStepsFromHomeCW = 0;
  jointState[motorIndex].maxEncoderStepsFromHomeCCW = 0;
  jointState[motorIndex].hasFoundHome = false;
  setGoal(motorIndex, GOAL_FIND_HOME);
}

// Start homing all motors
inline void startHomingAll() {
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    startHoming(i);
  }
}

// Forward declarations - implemented in .ino
void doGoal(int motorIndex);
void doAllGoals();

#endif // SCORBOT_PINS_H
