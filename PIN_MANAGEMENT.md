# Pin Management Guide

## Overview

All Scorbot pin assignments are now centralized in [scorbot.h](scorbot.h). This eliminates confusion and ensures all sketches use the same pin definitions.

## How to Use

In any Arduino sketch, simply include the header file:

```arduino
#include "../../scorbot.h"  // Adjust path as needed
```

### Option 1: Clean Struct Syntax (Recommended)

Access motor pins through the `SCORBOT[]` array:

```arduino
// Move base motor clockwise at 50% speed
digitalWrite(SCORBOT[MOTOR_BASE].dir1, HIGH);
digitalWrite(SCORBOT[MOTOR_BASE].dir2, LOW);
analogWrite(SCORBOT[MOTOR_BASE].pwm, 128);

// Check shoulder home switch
bool atHome = (digitalRead(SCORBOT[MOTOR_SHOULDER].home_switch) == LOW);

// Read elbow encoder
int p0 = digitalRead(SCORBOT[MOTOR_ELBOW].encoder_p0);
int p1 = digitalRead(SCORBOT[MOTOR_ELBOW].encoder_p1);

// Print motor name for debugging
Serial.println(SCORBOT[MOTOR_BASE].name);  // Prints "Base"
```

### Option 2: Direct Constants (Legacy)

Use the individual constants directly:

```arduino
digitalWrite(BASE_MOTOR_DIR1, HIGH);
analogWrite(BASE_MOTOR_PWM, 150);
int switchState = digitalRead(BASE_SWITCH);
```

## Pin Definitions Available

### Individual Motor Constants

- `BASE_MOTOR_DIR1`, `BASE_MOTOR_DIR2`, `BASE_MOTOR_PWM`
- `BASE_ENCODER_P0`, `BASE_ENCODER_P1`
- `BASE_SWITCH`

(Similar for SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER)

### Helper Arrays (for multi-axis control)

- `MOTOR_DIR1_PINS[6]` - Array of all DIR1 pins
- `MOTOR_DIR2_PINS[6]` - Array of all DIR2 pins
- `MOTOR_PWM_PINS[6]` - Array of all PWM pins
- `ENCODER_P0_PINS[6]` - Array of all encoder P0 pins
- `ENCODER_P1_PINS[6]` - Array of all encoder P1 pins
- `SWITCH_PINS[6]` - Array of all microswitch pins
- `MOTOR_NAMES[6]` - String names for debugging
- `NUM_MOTORS` - Constant = 6

### Example: Loop Through All Motors

**New clean syntax:**

```arduino
#include "scorbot.h"

void setup() {
  // Configure all motor pins with clean struct syntax
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(SCORBOT[i].dir1, OUTPUT);
    pinMode(SCORBOT[i].dir2, OUTPUT);
    pinMode(SCORBOT[i].pwm, OUTPUT);
    pinMode(SCORBOT[i].home_switch, INPUT_PULLUP);
    pinMode(SCORBOT[i].encoder_p0, INPUT_PULLUP);
    pinMode(SCORBOT[i].encoder_p1, INPUT_PULLUP);

    Serial.print("Configured motor: ");
    Serial.println(SCORBOT[i].name);  // Name included in struct!
  }
}

// Move any motor with a clean function
void moveMotor(int motorIndex, int speed, bool clockwise) {
  if (clockwise) {
    digitalWrite(SCORBOT[motorIndex].dir1, HIGH);
    digitalWrite(SCORBOT[motorIndex].dir2, LOW);
  } else {
    digitalWrite(SCORBOT[motorIndex].dir1, LOW);
    digitalWrite(SCORBOT[motorIndex].dir2, HIGH);
  }
  analogWrite(SCORBOT[motorIndex].pwm, speed);
}
```

**Legacy array syntax (still works):**

```arduino
void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_DIR1_PINS[i], OUTPUT);
    pinMode(MOTOR_DIR2_PINS[i], OUTPUT);
    pinMode(MOTOR_PWM_PINS[i], OUTPUT);
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);

    Serial.print("Configured motor: ");
    Serial.println(MOTOR_NAMES[i]);
  }
}
```

## Important: Interrupt Pin Limitation

⚠️ **CRITICAL ISSUE**: The current pin assignments have encoders on pins 34-45, which are **NOT interrupt-capable** on Arduino Mega.

### Interrupt-Capable Pins on Mega:

- Pins: 2, 3, 18, 19, 20, 21
- Problem: Pins 2-7 are used for PWM motor control
- Problem: Pins 18-19 are used for Serial1 communication

### Solutions:

#### Option 1: Use Polling (Implemented)

- File: `base_motor_test_polling.ino`
- Calls `updateEncoder()` in main loop instead of interrupts
- Works well for moderate motor speeds
- Simpler, no rewiring needed
- ✅ **Recommended to start**

#### Option 2: Rewire for Interrupts (Future)

- Move encoder pins to 20, 21 (for one motor)
- Requires updating DB-50 wiring and scorbot.h
- Better for high-speed operation
- More complex hardware change

#### Option 3: Use External Interrupt Expander

- Add hardware like PCF8574 or MCP23017
- Provides more interrupt pins
- Requires additional components

## Updating Pin Assignments

If you need to change pin assignments:

1. **Edit only [scorbot.h](scorbot.h)**
2. All sketches that include it will automatically use new pins
3. No need to update multiple files
4. Update comments to explain changes
5. Test each motor individually after changes

## Benefits of This Approach

✅ **Single source of truth** - One file to maintain
✅ **Consistency** - All sketches use same pins
✅ **Documentation** - Comments explain DB-50 mapping
✅ **Scalability** - Easy to add new motors or features
✅ **Arrays** - Simplify multi-axis control
✅ **Less error-prone** - Change once, affects all code

## File Locations

```
scorbot_controller/
├── scorbot.h              ← Main pin definitions (EDIT THIS)
├── helpers/
│   ├── base_motor_test/
│   │   ├── base_motor_test.ino          (interrupt version - broken on current pins)
│   │   └── base_motor_test_polling.ino  (polling version - works!)
│   ├── encoder_test/
│   │   └── encoder_test.ino
│   └── motor_only_test/
│       └── motor_only_test.ino
└── README.md                    ← High-level documentation only
```

## Migration from README.md

Previously, pin assignments were documented in README.md Table 1. This was documentation-only and required manually copying values into code.

Now:

- **scorbot.h** = Source of truth (code)
- **README.md** = High-level project documentation
- All sketches #include scorbot.h for consistency

If you find a pin assignment error:

1. Fix it in scorbot.h
2. Optionally update README.md if you want docs to match
3. Re-upload affected sketches
