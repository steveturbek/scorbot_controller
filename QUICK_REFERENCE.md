# Scorbot Pin Access - Quick Reference

## Include the Header

```arduino
#include "scorbot.h"
```

## Access Motor Pins (Recommended Syntax)

### Basic Access Pattern

```arduino
SCORBOT[motor_index].pin_name
```

### Motor Indices (Use These!)

```arduino
MOTOR_BASE          // 0 - Base rotation (310°)
MOTOR_SHOULDER      // 1 - Shoulder (+130° / -35°)
MOTOR_ELBOW         // 2 - Elbow (±130°)
MOTOR_WRIST_PITCH   // 3 - Wrist pitch (±130°)
MOTOR_WRIST_ROLL    // 4 - Wrist roll (unlimited)
MOTOR_GRIPPER       // 5 - Gripper
NUM_MOTORS          // 6 - Total count
```

### Available Pin Names

```arduino
.dir1          // Direction pin 1 (H-bridge IN1/IN3)
.dir2          // Direction pin 2 (H-bridge IN2/IN4)
.pwm           // PWM speed control (H-bridge ENA/ENB)
.encoder_p0    // Encoder channel P0
.encoder_p1    // Encoder channel P1
.home_switch   // Home microswitch
.name          // Motor name string (for Serial.print)
```

## Common Patterns

### Move Motor Clockwise

```arduino
digitalWrite(SCORBOT[MOTOR_BASE].dir1, HIGH);
digitalWrite(SCORBOT[MOTOR_BASE].dir2, LOW);
analogWrite(SCORBOT[MOTOR_BASE].pwm, 100);  // Speed 0-255
```

### Move Motor Counter-Clockwise

```arduino
digitalWrite(SCORBOT[MOTOR_BASE].dir1, LOW);
digitalWrite(SCORBOT[MOTOR_BASE].dir2, HIGH);
analogWrite(SCORBOT[MOTOR_BASE].pwm, 100);
```

### Stop Motor

```arduino
digitalWrite(SCORBOT[MOTOR_BASE].dir1, LOW);
digitalWrite(SCORBOT[MOTOR_BASE].dir2, LOW);
analogWrite(SCORBOT[MOTOR_BASE].pwm, 0);
```

### Check Home Switch (Active LOW)

```arduino
bool atHome = (digitalRead(SCORBOT[MOTOR_BASE].home_switch) == LOW);
if (atHome) {
  Serial.println("At home position!");
}
```

### Read Encoder

```arduino
int p0 = digitalRead(SCORBOT[MOTOR_BASE].encoder_p0);
int p1 = digitalRead(SCORBOT[MOTOR_BASE].encoder_p1);
```

### Print Motor Name (Debugging)

```arduino
Serial.print("Moving ");
Serial.println(SCORBOT[MOTOR_BASE].name);  // Prints "Base"
```

## Generic Motor Control Function

```arduino
void moveMotor(int motor, int speed, bool clockwise) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  digitalWrite(SCORBOT[motor].dir1, clockwise ? HIGH : LOW);
  digitalWrite(SCORBOT[motor].dir2, clockwise ? LOW : HIGH);
  analogWrite(SCORBOT[motor].pwm, speed);

  Serial.print(SCORBOT[motor].name);
  Serial.println(clockwise ? " CW" : " CCW");
}

void stopMotor(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  digitalWrite(SCORBOT[motor].dir1, LOW);
  digitalWrite(SCORBOT[motor].dir2, LOW);
  analogWrite(SCORBOT[motor].pwm, 0);
}

// Usage:
moveMotor(MOTOR_BASE, 100, true);   // Base CW at speed 100
delay(2000);
stopMotor(MOTOR_BASE);
```

## Loop Through All Motors

```arduino
void setup() {
  // Configure all 6 motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(SCORBOT[i].dir1, OUTPUT);
    pinMode(SCORBOT[i].dir2, OUTPUT);
    pinMode(SCORBOT[i].pwm, OUTPUT);
    pinMode(SCORBOT[i].encoder_p0, INPUT_PULLUP);
    pinMode(SCORBOT[i].encoder_p1, INPUT_PULLUP);
    pinMode(SCORBOT[i].home_switch, INPUT_PULLUP);

    Serial.print("Configured: ");
    Serial.println(SCORBOT[i].name);
  }
}
```

## Alternative: Direct Constants (Legacy)

If you prefer the old style, these still work:

```arduino
BASE_MOTOR_DIR1, BASE_MOTOR_DIR2, BASE_MOTOR_PWM
BASE_ENCODER_P0, BASE_ENCODER_P1
BASE_SWITCH

// Similar for: SHOULDER_, ELBOW_, WRIST_PITCH_, WRIST_ROLL_, GRIPPER_
```

## Example: Multi-Motor Homing

```arduino
void homeMotor(int motor) {
  Serial.print("Homing ");
  Serial.println(SCORBOT[motor].name);

  // Move slowly CCW until switch pressed or timeout
  digitalWrite(SCORBOT[motor].dir1, LOW);
  digitalWrite(SCORBOT[motor].dir2, HIGH);
  analogWrite(SCORBOT[motor].pwm, 50);

  unsigned long start = millis();
  while (digitalRead(SCORBOT[motor].home_switch) == HIGH) {
    if (millis() - start > 10000) {
      Serial.println("Timeout!");
      stopMotor(motor);
      return;
    }
  }

  stopMotor(motor);
  Serial.println("Home found!");
}

void loop() {
  // Home all motors sequentially
  for (int i = 0; i < NUM_MOTORS; i++) {
    homeMotor(i);
    delay(1000);
  }
}
```

## See Also

- [scorbot.h](scorbot.h) - Full pin definitions
- [PIN_MANAGEMENT.md](PIN_MANAGEMENT.md) - Detailed guide
- [helpers/multi_axis_example/](helpers/multi_axis_example/) - Working example sketch
