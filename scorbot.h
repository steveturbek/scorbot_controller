/*
  SCORBOT ER III - PIN DEFINITIONS

  This file defines all Arduino Mega pin assignments for the Scorbot robot.
  Include this file in any sketch to ensure consistent pin usage.

  Based on DB-50 connector wiring and L298N motor driver configuration.
  Last updated: 2026-01-08
*/

#ifndef SCORBOT_PINS_H
#define SCORBOT_PINS_H

#include <string.h>  // For strcmp() function

// ============================================================================
// MOTOR 1: BASE (310° rotation)
// ============================================================================
// L298N Board 1, Motor A (Output 1)
#define BASE_MOTOR_NAME        "base"
#define BASE_MOTOR_MIN            40
#define BASE_MOTOR_CCW_PIN        22    // DB-50 pin 17 -> L298N IN1
#define BASE_MOTOR_CW_PIN         23    // DB-50 pin 50 -> L298N IN2
#define BASE_MOTOR_PWM_PIN        7     // PWM speed control -> L298N ENA
#define BASE_ENCODER_P0_PIN       34    // DB-50 pin 2
#define BASE_ENCODER_P1_PIN       35    // DB-50 pin 5
#define BASE_HOME_SWITCH_PIN      46    // DB-50 pin 23 (home microswitch)

// ============================================================================
// MOTOR 2: SHOULDER (+130° / -35°)
// ============================================================================
// L298N Board 1, Motor B (Output 2)
#define SHOULDER_MOTOR_NAME   "shoulder"
#define SHOULDER_MOTOR_CCW_PIN    24    // DB-50 pin 16 -> L298N IN3
#define SHOULDER_MOTOR_CW_PIN     25    // DB-50 pin 49 -> L298N IN4
#define SHOULDER_MOTOR_PWM_PIN    6     // PWM speed control -> L298N ENB
#define SHOULDER_ENCODER_P0_PIN   36    // DB-50 pin 1
#define SHOULDER_ENCODER_P1_PIN   37    // DB-50 pin 21
#define SHOULDER_HOME_SWITCH_PIN  47    // DB-50 pin 7 (home microswitch)

// ============================================================================
// MOTOR 3: ELBOW (±130°)
// ============================================================================
// L298N Board 2, Motor A (Output 1)
#define ELBOW_MOTOR_NAME         "elbow"
#define ELBOW_MOTOR_CCW_PIN       26    // DB-50 pin 15 -> L298N IN1
#define ELBOW_MOTOR_CW_PIN        27    // DB-50 pin 48 -> L298N IN2
#define ELBOW_MOTOR_PWM_PIN       5     // PWM speed control -> L298N ENA
#define ELBOW_ENCODER_P0_PIN      38    // DB-50 pin 36
#define ELBOW_ENCODER_P1_PIN      39    // DB-50 pin 4
#define ELBOW_HOME_SWITCH_PIN     48    // DB-50 pin 24 (home microswitch)

// ============================================================================
// MOTOR 4: WRIST PITCH (±130°)
// ============================================================================
// L298N Board 2, Motor B (Output 2)
#define WRIST_PITCH_MOTOR_NAME      "wrist_pitch"
#define WRIST_PITCH_MOTOR_CCW_PIN   28  // DB-50 pin 14 -> L298N IN3
#define WRIST_PITCH_MOTOR_CW_PIN    29  // DB-50 pin 47 -> L298N IN4
#define WRIST_PITCH_MOTOR_PWM_PIN   4   // PWM speed control -> L298N ENB
#define WRIST_PITCH_ENCODER_P0_PIN  40  // DB-50 pin 35
#define WRIST_PITCH_ENCODER_P1_PIN  41  // DB-50 pin 20
#define WRIST_PITCH_HOME_SWITCH_PIN      49  // DB-50 pin 8 (home microswitch)

// ============================================================================
// MOTOR 5: WRIST ROLL (unlimited rotation)
// ============================================================================
// L298N Board 3, Motor A (Output 1)
#define WRIST_ROLL_MOTOR_NAME        "wrist_roll"
#define WRIST_ROLL_MOTOR_CCW_PIN    30  // DB-50 pin 13 -> L298N IN1
#define WRIST_ROLL_MOTOR_CW_PIN     31  // DB-50 pin 46 -> L298N IN2
#define WRIST_ROLL_MOTOR_PWM_PIN    3   // PWM speed control -> L298N ENA
#define WRIST_ROLL_ENCODER_P0_PIN   42  // DB-50 pin 18
#define WRIST_ROLL_ENCODER_P1_PIN   43  // DB-50 pin 3
#define WRIST_ROLL_HOME_SWITCH_PIN       50  // DB-50 pin 6 (home microswitch)

// ============================================================================
// MOTOR 6: GRIPPER
// ============================================================================
// L298N Board 3, Motor B (Output 2)
#define GRIPPER_MOTOR_NAME          "gripper"
#define GRIPPER_MOTOR_CCW_PIN       32  // DB-50 pin 12 -> L298N IN3
#define GRIPPER_MOTOR_CW_PIN        33  // DB-50 pin 45 -> L298N IN4
#define GRIPPER_MOTOR_PWM_PIN       2   // PWM speed control -> L298N ENB
#define GRIPPER_ENCODER_P0_PIN      44  // DB-50 pin 34
#define GRIPPER_ENCODER_P1_PIN      45  // DB-50 pin 19
#define GRIPPER_HOME_SWITCH_PIN          51  // DB-50 pin 22 (manual says not connected - needs testing)

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================
// Note: Serial1 uses pins 18 (TX) and 19 (RX) on Arduino Mega
// These are used for communication with external joystick controller
#define SERIAL_TX               18  // Reserved for Serial1
#define SERIAL_RX               19  // Reserved for Serial1

// ============================================================================
// INTERRUPT-CAPABLE PINS ON ARDUINO MEGA
// ============================================================================
// Available interrupt pins: 2, 3, 18, 19, 20, 21
// Note: Pins 18, 19 are used by Serial1
// Note: Pins 2-7 are used for PWM motor control
// WARNING: Current encoder pins (34-45) are NOT interrupt-capable!
//          Use polling method for encoder reading, or rewire to interrupt pins

// ============================================================================
// MOTOR AXIS STRUCTURE (for clean, readable code)
// ============================================================================

struct MotorPins {
  int CCW;           // Direction pin (H-bridge IN1 or IN3)
  int CW;              // Direction pin (H-bridge IN2 or IN4)
  int pwm;           // PWM speed control pin (H-bridge ENA or ENB)
  int encoder_p0;    // Encoder channel P0
  int encoder_p1;    // Encoder channel P1
  int home_switch;   // Home microswitch pin
  const char* name;  // Motor name for debugging
};

// Motor axis indices (for array access)
enum MotorIndex {
  MOTOR_BASE = 0,
  MOTOR_SHOULDER,
  MOTOR_ELBOW,
  MOTOR_WRIST_PITCH,
  MOTOR_WRIST_ROLL,
  MOTOR_GRIPPER,
  MotorIndex_COUNT // this is the count of the enum 
};

// Main Scorbot motor configuration array
// Usage: SCORBOT[MOTOR_BASE].pwm or SCORBOT[0].pwm
// Using array designators to prevent enum/array order mismatch
const MotorPins SCORBOT[MotorIndex_COUNT] = {
  [MOTOR_BASE] = {
    .CCW = BASE_MOTOR_CCW_PIN,
    .CW = BASE_MOTOR_CW_PIN,
    .pwm = BASE_MOTOR_PWM_PIN,
    .encoder_p0 = BASE_ENCODER_P0_PIN,
    .encoder_p1 = BASE_ENCODER_P1_PIN,
    .home_switch = BASE_HOME_SWITCH_PIN,
    .name = BASE_MOTOR_NAME
  },
  [MOTOR_SHOULDER] = {
    .CCW = SHOULDER_MOTOR_CCW_PIN,
    .CW = SHOULDER_MOTOR_CW_PIN,
    .pwm = SHOULDER_MOTOR_PWM_PIN,
    .encoder_p0 = SHOULDER_ENCODER_P0_PIN,
    .encoder_p1 = SHOULDER_ENCODER_P1_PIN,
    .home_switch = SHOULDER_HOME_SWITCH_PIN,
    .name = SHOULDER_MOTOR_NAME
  },
  [MOTOR_ELBOW] = {
    .CCW = ELBOW_MOTOR_CCW_PIN,
    .CW = ELBOW_MOTOR_CW_PIN,
    .pwm = ELBOW_MOTOR_PWM_PIN,
    .encoder_p0 = ELBOW_ENCODER_P0_PIN,
    .encoder_p1 = ELBOW_ENCODER_P1_PIN,
    .home_switch = ELBOW_HOME_SWITCH_PIN,
    .name = ELBOW_MOTOR_NAME
  },
  [MOTOR_WRIST_PITCH] = {
    .CCW = WRIST_PITCH_MOTOR_CCW_PIN,
    .CW = WRIST_PITCH_MOTOR_CW_PIN,
    .pwm = WRIST_PITCH_MOTOR_PWM_PIN,
    .encoder_p0 = WRIST_PITCH_ENCODER_P0_PIN,
    .encoder_p1 = WRIST_PITCH_ENCODER_P1_PIN,
    .home_switch = WRIST_PITCH_HOME_SWITCH_PIN,
    .name = WRIST_PITCH_MOTOR_NAME
  },
  [MOTOR_WRIST_ROLL] = {
    .CCW = WRIST_ROLL_MOTOR_CCW_PIN,
    .CW = WRIST_ROLL_MOTOR_CW_PIN,
    .pwm = WRIST_ROLL_MOTOR_PWM_PIN,
    .encoder_p0 = WRIST_ROLL_ENCODER_P0_PIN,
    .encoder_p1 = WRIST_ROLL_ENCODER_P1_PIN,
    .home_switch = WRIST_ROLL_HOME_SWITCH_PIN,
    .name = WRIST_ROLL_MOTOR_NAME
  },
  [MOTOR_GRIPPER] = {
    .CCW = GRIPPER_MOTOR_CCW_PIN,
    .CW = GRIPPER_MOTOR_CW_PIN,
    .pwm = GRIPPER_MOTOR_PWM_PIN,
    .encoder_p0 = GRIPPER_ENCODER_P0_PIN,
    .encoder_p1 = GRIPPER_ENCODER_P1_PIN,
    .home_switch = GRIPPER_HOME_SWITCH_PIN,
    .name = GRIPPER_MOTOR_NAME
  }
};

// ============================================================================
// HELPER FUNCTION DECLARATIONS (implement these in your sketch)
// ============================================================================

// Configure all pins for a single motor
// Usage: setupMotor(MOTOR_BASE);
inline void setupMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= MotorIndex_COUNT) return;

  pinMode(SCORBOT[motorIndex].CCW, OUTPUT);
  pinMode(SCORBOT[motorIndex].CW, OUTPUT);
  pinMode(SCORBOT[motorIndex].pwm, OUTPUT);
  pinMode(SCORBOT[motorIndex].encoder_p0, INPUT_PULLUP);
  pinMode(SCORBOT[motorIndex].encoder_p1, INPUT_PULLUP);
  pinMode(SCORBOT[motorIndex].home_switch, INPUT_PULLUP);
}

// Configure all 6 motors at once
// Usage: setupAllMotors();
inline void setupAllMotors() {
  for (int i = 0; i < MotorIndex_COUNT; i++) {
    setupMotor(i);
  }
}

// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= MotorIndex_COUNT) return;

  digitalWrite(SCORBOT[motorIndex].CCW, LOW);
  digitalWrite(SCORBOT[motorIndex].CW, LOW);
  analogWrite(SCORBOT[motorIndex].pwm, 0);
}

// Move a motor
// Usage: moveMotor(MOTOR_BASE, 50, true);  // turn CW at speed 50% (range: 0-99)
inline void moveMotor(int motorIndex, int speed, bool clockwise) {
  if (motorIndex < 0 || motorIndex >= MotorIndex_COUNT) return;

  digitalWrite(SCORBOT[motorIndex].CCW, clockwise ? HIGH : LOW);
  digitalWrite(SCORBOT[motorIndex].CW, clockwise ? LOW : HIGH);

  // Convert normalized speed (0-99) to PWM value (0-255)
  int pwmValue = (speed * 255) / 99;
  analogWrite(SCORBOT[motorIndex].pwm, pwmValue);
}


#endif // SCORBOT_PINS_H
