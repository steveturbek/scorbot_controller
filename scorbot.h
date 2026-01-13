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




#endif // SCORBOT_PINS_H
