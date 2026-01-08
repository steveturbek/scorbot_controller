/*
  SCORBOT ER III - PIN DEFINITIONS

  This file defines all Arduino Mega pin assignments for the Scorbot robot.
  Include this file in any sketch to ensure consistent pin usage.

  Based on DB-50 connector wiring and L298N motor driver configuration.
  Last updated: 2026-01-08
*/

#ifndef SCORBOT_PINS_H
#define SCORBOT_PINS_H

// ============================================================================
// MOTOR 1: BASE (310° rotation)
// ============================================================================
// L298N Board 1, Motor A (Output 1)
#define BASE_MOTOR_DIR1       22    // DB-50 pin 17 -> L298N IN1
#define BASE_MOTOR_DIR2       23    // DB-50 pin 50 -> L298N IN2
#define BASE_MOTOR_PWM        7     // PWM speed control -> L298N ENA
#define BASE_ENCODER_P0       34    // DB-50 pin 2
#define BASE_ENCODER_P1       35    // DB-50 pin 5
#define BASE_SWITCH           46    // DB-50 pin 23 (home microswitch)

// ============================================================================
// MOTOR 2: SHOULDER (+130° / -35°)
// ============================================================================
// L298N Board 1, Motor B (Output 2)
#define SHOULDER_MOTOR_DIR1   24    // DB-50 pin 16 -> L298N IN3
#define SHOULDER_MOTOR_DIR2   25    // DB-50 pin 49 -> L298N IN4
#define SHOULDER_MOTOR_PWM    6     // PWM speed control -> L298N ENB
#define SHOULDER_ENCODER_P0   36    // DB-50 pin 1
#define SHOULDER_ENCODER_P1   37    // DB-50 pin 21
#define SHOULDER_SWITCH       47    // DB-50 pin 7 (home microswitch)

// ============================================================================
// MOTOR 3: ELBOW (±130°)
// ============================================================================
// L298N Board 2, Motor A (Output 1)
#define ELBOW_MOTOR_DIR1      26    // DB-50 pin 15 -> L298N IN1
#define ELBOW_MOTOR_DIR2      27    // DB-50 pin 48 -> L298N IN2
#define ELBOW_MOTOR_PWM       5     // PWM speed control -> L298N ENA
#define ELBOW_ENCODER_P0      38    // DB-50 pin 36
#define ELBOW_ENCODER_P1      39    // DB-50 pin 4
#define ELBOW_SWITCH          48    // DB-50 pin 24 (home microswitch)

// ============================================================================
// MOTOR 4: WRIST PITCH (±130°)
// ============================================================================
// L298N Board 2, Motor B (Output 2)
#define WRIST_PITCH_MOTOR_DIR1  28  // DB-50 pin 14 -> L298N IN3
#define WRIST_PITCH_MOTOR_DIR2  29  // DB-50 pin 47 -> L298N IN4
#define WRIST_PITCH_MOTOR_PWM   4   // PWM speed control -> L298N ENB
#define WRIST_PITCH_ENCODER_P0  40  // DB-50 pin 35
#define WRIST_PITCH_ENCODER_P1  41  // DB-50 pin 20
#define WRIST_PITCH_SWITCH      49  // DB-50 pin 8 (home microswitch)

// ============================================================================
// MOTOR 5: WRIST ROLL (unlimited rotation)
// ============================================================================
// L298N Board 3, Motor A (Output 1)
#define WRIST_ROLL_MOTOR_DIR1   30  // DB-50 pin 13 -> L298N IN1
#define WRIST_ROLL_MOTOR_DIR2   31  // DB-50 pin 46 -> L298N IN2
#define WRIST_ROLL_MOTOR_PWM    3   // PWM speed control -> L298N ENA
#define WRIST_ROLL_ENCODER_P0   42  // DB-50 pin 18
#define WRIST_ROLL_ENCODER_P1   43  // DB-50 pin 3
#define WRIST_ROLL_SWITCH       50  // DB-50 pin 6 (home microswitch)

// ============================================================================
// MOTOR 6: GRIPPER
// ============================================================================
// L298N Board 3, Motor B (Output 2)
#define GRIPPER_MOTOR_DIR1      32  // DB-50 pin 12 -> L298N IN3
#define GRIPPER_MOTOR_DIR2      33  // DB-50 pin 45 -> L298N IN4
#define GRIPPER_MOTOR_PWM       2   // PWM speed control -> L298N ENB
#define GRIPPER_ENCODER_P0      44  // DB-50 pin 34
#define GRIPPER_ENCODER_P1      45  // DB-50 pin 19
#define GRIPPER_SWITCH          51  // DB-50 pin 22 (manual says not connected - needs testing)

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
  int dir1;          // Direction pin 1 (H-bridge IN1 or IN3)
  int dir2;          // Direction pin 2 (H-bridge IN2 or IN4)
  int pwm;           // PWM speed control pin (H-bridge ENA or ENB)
  int encoder_p0;    // Encoder channel P0
  int encoder_p1;    // Encoder channel P1
  int home_switch;   // Home microswitch pin
  const char* name;  // Motor name for debugging
};

// Motor axis indices (for array access)
enum MotorIndex {
  MOTOR_BASE = 0,
  MOTOR_SHOULDER = 1,
  MOTOR_ELBOW = 2,
  MOTOR_WRIST_PITCH = 3,
  MOTOR_WRIST_ROLL = 4,
  MOTOR_GRIPPER = 5,
  NUM_MOTORS = 6
};

// Main Scorbot motor configuration array
// Usage: SCORBOT[MOTOR_BASE].pwm or SCORBOT[0].pwm
const MotorPins SCORBOT[NUM_MOTORS] = {
  // Base motor
  {
    .dir1 = BASE_MOTOR_DIR1,
    .dir2 = BASE_MOTOR_DIR2,
    .pwm = BASE_MOTOR_PWM,
    .encoder_p0 = BASE_ENCODER_P0,
    .encoder_p1 = BASE_ENCODER_P1,
    .home_switch = BASE_SWITCH,
    .name = "Base"
  },
  // Shoulder motor
  {
    .dir1 = SHOULDER_MOTOR_DIR1,
    .dir2 = SHOULDER_MOTOR_DIR2,
    .pwm = SHOULDER_MOTOR_PWM,
    .encoder_p0 = SHOULDER_ENCODER_P0,
    .encoder_p1 = SHOULDER_ENCODER_P1,
    .home_switch = SHOULDER_SWITCH,
    .name = "Shoulder"
  },
  // Elbow motor
  {
    .dir1 = ELBOW_MOTOR_DIR1,
    .dir2 = ELBOW_MOTOR_DIR2,
    .pwm = ELBOW_MOTOR_PWM,
    .encoder_p0 = ELBOW_ENCODER_P0,
    .encoder_p1 = ELBOW_ENCODER_P1,
    .home_switch = ELBOW_SWITCH,
    .name = "Elbow"
  },
  // Wrist Pitch motor
  {
    .dir1 = WRIST_PITCH_MOTOR_DIR1,
    .dir2 = WRIST_PITCH_MOTOR_DIR2,
    .pwm = WRIST_PITCH_MOTOR_PWM,
    .encoder_p0 = WRIST_PITCH_ENCODER_P0,
    .encoder_p1 = WRIST_PITCH_ENCODER_P1,
    .home_switch = WRIST_PITCH_SWITCH,
    .name = "Wrist Pitch"
  },
  // Wrist Roll motor
  {
    .dir1 = WRIST_ROLL_MOTOR_DIR1,
    .dir2 = WRIST_ROLL_MOTOR_DIR2,
    .pwm = WRIST_ROLL_MOTOR_PWM,
    .encoder_p0 = WRIST_ROLL_ENCODER_P0,
    .encoder_p1 = WRIST_ROLL_ENCODER_P1,
    .home_switch = WRIST_ROLL_SWITCH,
    .name = "Wrist Roll"
  },
  // Gripper motor
  {
    .dir1 = GRIPPER_MOTOR_DIR1,
    .dir2 = GRIPPER_MOTOR_DIR2,
    .pwm = GRIPPER_MOTOR_PWM,
    .encoder_p0 = GRIPPER_ENCODER_P0,
    .encoder_p1 = GRIPPER_ENCODER_P1,
    .home_switch = GRIPPER_SWITCH,
    .name = "Gripper"
  }
};

// ============================================================================
// LEGACY ARRAYS (for backward compatibility)
// ============================================================================
// These are kept for any existing code that uses them

const int MOTOR_DIR1_PINS[] = {
  BASE_MOTOR_DIR1, SHOULDER_MOTOR_DIR1, ELBOW_MOTOR_DIR1,
  WRIST_PITCH_MOTOR_DIR1, WRIST_ROLL_MOTOR_DIR1, GRIPPER_MOTOR_DIR1
};

const int MOTOR_DIR2_PINS[] = {
  BASE_MOTOR_DIR2, SHOULDER_MOTOR_DIR2, ELBOW_MOTOR_DIR2,
  WRIST_PITCH_MOTOR_DIR2, WRIST_ROLL_MOTOR_DIR2, GRIPPER_MOTOR_DIR2
};

const int MOTOR_PWM_PINS[] = {
  BASE_MOTOR_PWM, SHOULDER_MOTOR_PWM, ELBOW_MOTOR_PWM,
  WRIST_PITCH_MOTOR_PWM, WRIST_ROLL_MOTOR_PWM, GRIPPER_MOTOR_PWM
};

const int ENCODER_P0_PINS[] = {
  BASE_ENCODER_P0, SHOULDER_ENCODER_P0, ELBOW_ENCODER_P0,
  WRIST_PITCH_ENCODER_P0, WRIST_ROLL_ENCODER_P0, GRIPPER_ENCODER_P0
};

const int ENCODER_P1_PINS[] = {
  BASE_ENCODER_P1, SHOULDER_ENCODER_P1, ELBOW_ENCODER_P1,
  WRIST_PITCH_ENCODER_P1, WRIST_ROLL_ENCODER_P1, GRIPPER_ENCODER_P1
};

const int SWITCH_PINS[] = {
  BASE_SWITCH, SHOULDER_SWITCH, ELBOW_SWITCH,
  WRIST_PITCH_SWITCH, WRIST_ROLL_SWITCH, GRIPPER_SWITCH
};

const char* MOTOR_NAMES[] = {
  "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"
};

// ============================================================================
// HELPER FUNCTION DECLARATIONS (implement these in your sketch)
// ============================================================================

// Configure all pins for a single motor
// Usage: setupMotor(MOTOR_BASE);
inline void setupMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  pinMode(SCORBOT[motorIndex].dir1, OUTPUT);
  pinMode(SCORBOT[motorIndex].dir2, OUTPUT);
  pinMode(SCORBOT[motorIndex].pwm, OUTPUT);
  pinMode(SCORBOT[motorIndex].encoder_p0, INPUT_PULLUP);
  pinMode(SCORBOT[motorIndex].encoder_p1, INPUT_PULLUP);
  pinMode(SCORBOT[motorIndex].home_switch, INPUT_PULLUP);
}

// Configure all 6 motors at once
// Usage: setupAllMotors();
inline void setupAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    setupMotor(i);
  }
}

// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  digitalWrite(SCORBOT[motorIndex].dir1, LOW);
  digitalWrite(SCORBOT[motorIndex].dir2, LOW);
  analogWrite(SCORBOT[motorIndex].pwm, 0);
}

// Move a motor
// Usage: moveMotor(MOTOR_BASE, 100, true);  // CW at speed 100
inline void moveMotor(int motorIndex, int speed, bool clockwise) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  digitalWrite(SCORBOT[motorIndex].dir1, clockwise ? HIGH : LOW);
  digitalWrite(SCORBOT[motorIndex].dir2, clockwise ? LOW : HIGH);
  analogWrite(SCORBOT[motorIndex].pwm, speed);
}

#endif // SCORBOT_PINS_H
