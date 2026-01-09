# SCORBOT Controller Project Notes

## Analysis of GitHub Projects for Arduino Mega Implementation

**Date:** January 2026  
**Purpose:** Strategic guidance for Scorbot ER-III rebuild with Arduino Mega

---

## Executive Summary

### Gripper Home Switch Answer ✓

**The ER-III gripper does NOT have a home switch.**

From manual (Table D-1, pin 22): "Gripper Microswitch - manual says not connected"

From amiravni's blog: "Six more pins are connected to the microswitches but in fact there are only 5, the gripper does not have a microswitch but there's a pin kept for that option."

### Gripper Calibration Strategies Found:

1. **Current sensing method** - Close until motor stalls (controlled current limit)
2. **Encoder-based soft limits** - Store min/max values after manual calibration
3. **Zero at fully closed** - Set position to 0 when fully closed, track from there
4. **Use encoder counts** - Track relative position from known state

---

## Project Analysis

### 1. **amiravni/Scorbot** MOST RELEVANT

**Platform:** Arduino MEGA + L293D drivers  
**Scorbot Model:** ER-V (very similar to ER-III)  
**Status:** Working, used for drawing

#### Key Technical Decisions:

- **Encoder reading:** Timer-based interrupt polling (~300ns per check)
- **Motor control:** Simple PWM through L293D (no PID initially)
- **Homing:** Microswitch interrupts zero encoder counters
- **Gripper:** No home switch, encoder-based positioning
- **Communication:** Serial commands from Python
- **Custom PCB shield:** Created for clean wiring

#### Code Architecture:

```
ScorbotCOMcontrol/ - Main controller (serial commands)
Encoder_Scorbot/   - Modified encoder library
python/            - Drawing control software
```

#### Critical Insights:

- "For each encoder we define a counter, and when the interrupt is activate (occurs about every ~300ns) it executes a function which checks the 6 encoders P0 and P1 pins"
- Encoder library: Polls all 6 encoders in single fast interrupt
- Microswitches trigger interrupts to zero position
- Successfully controlled 6 motors + drew complex images

#### Limitations Noted:

- No PID control initially (later versions may have added)
- Simple open-loop position control
- No trajectory planning

---

### 2. **ethanleep/robotic-arm (via usermanual.wiki)**

**Platform:** Arduino MEGA + STM32 "Blue Pill" + L239D drivers  
**Scorbot Model:** ER-III  
**Status:** Working controller with joystick interface

#### Key Technical Decisions:

- **Architecture:** Distributed - STM32 for each motor pair
- **Communication:** I2C between main controller and motor boards
- **Motor control:** L239D H-bridges
- **Encoders:** Filter circuitry added to clean signals
- **Joystick control:** Limit switch-based joysticks (5V when active)

#### Code Architecture:

- Main controller: USB communication + I2C coordinator
- Motor boards: Independent encoder reading + motor control
- Breakout board: Custom PCB for DB-50 connector

#### Critical Insights:

- Motor current: ~400mA normal operation at 12V
- Encoder LEDs: 2.3V requirement
- Only axes 0 and 2 used limit switches (partial homing)
- Power supply: 14V DC main, 2.4V for LEDs, 5V for logic

#### Advantages:

- Distributed processing reduces Arduino load
- Filter circuitry improves encoder reliability
- Joystick interface for manual control

---

### 3. **luisp23/Scorbot**

**Platform:** Arduino-based joystick controller  
**Scorbot Model:** ER-III or similar  
**Status:** Alternative to expensive controller

#### Key Insights:

- Focused on joystick control as primary interface
- Cost-effective replacement strategy
- ECE project from Georgia Tech
- Minimal documentation but working prototype

---

### 4. **aamitn/Scorbot-ER-V-Simulator-Controller** PROFESSIONAL GRADE

**Platform:** Arduino + MATLAB + ROS  
**Scorbot Model:** ER-V+  
**Status:** Complete system with simulation

#### Key Technical Decisions:

- **Motor control:** PID position control
- **Kinematics:** Full forward/inverse kinematics in MATLAB
- **Safety:** Industrial-grade emergency stop system
- **Interface:** Teach pendant + computer control
- **Simulation:** Complete MATLAB simulator before hardware

#### Code Architecture (MATLAB):

```matlab
@scorbot/Scorbot.m - Main class
SSCdirectModel()   - Forward kinematics
SSCmotorsToThetas() - Motor to joint conversion
SSCthetasToMotors() - Joint to motor conversion
```

#### Critical Insights:

- PID control essential for accuracy
- Separate motor/joint coordinate systems
- Gripper sensor support for force control
- Display system for real-time feedback
- Uses ROS for advanced control

#### Professional Features:

- Error handling with diagnostic LEDs
- Integrated display showing status
- Serial interface for programming
- Safety interlocks throughout

---

### 5. **jamofer/OpenScorbot**

**Platform:** BeagleBone (HRobot) + LPC1768 (LRobot)  
**Scorbot Model:** ER-IX  
**Status:** Complete hardware/software replacement

#### Key Insights:

- Two-tier architecture: High-level + Low-level
- LPC1768 for real-time motor control
- BeagleBone for kinematics/planning
- Noted as "poor code quality" by author (learning project)

---

### 6. **kutzer/ScorBotToolbox** BEST DOCUMENTATION

**Platform:** MATLAB toolbox  
**Scorbot Model:** ER-4U  
**Status:** Production-ready, USNA educational use

#### Key Insights:

- Comprehensive visualization tools
- Extensive error checking
- Well-documented API
- Requires original Intelitek controller
- Excellent reference for kinematics equations

---

### 7. **tidus747/openScorbot**

**Platform:** Python 3.6.9 + Qt 5 GUI  
**Scorbot Model:** ER-4U  
**Status:** Free controller with GUI

#### Key Insights:

- High-level language (Python) for control
- GUI-based interface
- Focuses on user-level control
- No low-level kinematics

---

### 8. **Hackaday.io ROBOT ARM project**

**Platform:** Arduino MEGA + Raspberry Pi + LM298 drivers  
**Scorbot Model:** ER-V  
**Status:** Shelved but functional

#### Key Technical Decisions:

- **Motor control:** "Terribly-tuned PID loops"
- **Kinematics:** Full forward/inverse kinematics
- **Interface:** G-code-like text commands
- **Architecture:** Arduino for motor control, Pi for planning

#### Critical Quote:

- "an underpowered microcontroller to run the kinematics"
- Used Arduino PID library (modified)
- Linear interpolation for smooth paths
- Linear speed control

#### Lessons Learned:

- Arduino MEGA struggles with kinematics calculations
- PID tuning is difficult and critical
- Separate planning/execution is beneficial

---

## Motor Control Strategies Summary

### Simple PWM (No PID)

**Used by:** amiravni initial version  
**Pros:** Simple, fast response  
**Cons:** No accuracy, overshoot, oscillation  
**Best for:** Initial testing, manual control

### Position PID Control

**Used by:** aamitn, Hackaday project  
**Pros:** Accurate positioning, smooth motion  
**Cons:** Requires tuning, computational overhead  
**Best for:** Autonomous operation, precision tasks

### Distributed Control

**Used by:** ethanleep  
**Pros:** Offloads processing, modular  
**Cons:** Complex wiring, multiple boards  
**Best for:** Professional builds

---

## Encoder Reading Strategies

### 1. **Fast Timer Interrupt (amiravni)**

```cpp
// Poll all encoders in single interrupt ~300ns
ISR(TIMER_OVF) {
    for each encoder:
        check P0, P1 pins
        update counter (+1, -1, or 0)
}
```

**Pros:** Single interrupt, efficient  
**Cons:** Might miss fast pulses

### 2. **Pin Change Interrupts (Standard Arduino)**

```cpp
// Separate interrupt per encoder channel
attachInterrupt(P0_pin, encoder_ISR, CHANGE);
```

**Pros:** Never misses pulses  
**Cons:** Many interrupts, overhead

### 3. **Polling in Main Loop**

**Pros:** Simple, no interrupts  
**Cons:** Slow, unreliable, NOT RECOMMENDED

### Recommendation:

Use Arduino Encoder library with hardware interrupts for critical axes (base, shoulder), timer interrupts for others.

---

## Homing Strategies

### Full Home Search (5 axes)

```
1. Move each axis slowly until microswitch triggers
2. Back off slightly
3. Approach slowly again for accuracy
4. Set encoder position to 0
5. Move to defined home position
```

### Partial Home (ethanleep approach)

- Only home base (axis 1) and shoulder (axis 2)
- Other axes use relative positioning
- **Risk:** Cumulative error over time

### No Home Option

- Start from arbitrary position
- Store position to EEPROM on shutdown
- Restore on startup
- **Risk:** Lost if power fails during operation

### Recommendation:

Full home search on startup. All 5 axes have working microswitches in your robot.

---

## Gripper Calibration (No Home Switch)

### Method 1: Current Sensing

```cpp
void calibrateGripper() {
    closeGripperUntilStall();  // Monitor current
    encoderCount_gripper = 0;   // This is fully closed
    openGripper(MAX_OPEN);      // Known max opening
    // Now have calibrated range
}
```

### Method 2: Manual Calibration

```cpp
// User manually positions gripper
// Store encoder values
int gripperMinPosition = 0;      // Fully closed
int gripperMaxPosition = 2500;   // Fully open (measured)
```

### Method 3: Soft Limits from Encoder

```cpp
// First run: discover limits
while (movingGripper) {
    if (encoderNotChanging && motorActive) {
        // Hit limit, record position
    }
}
```

### Recommendation:

Use Method 1 (current sensing) with low current limit for safety.

---

## Communication Protocols

### Serial Commands (Most Common)

```
Format: "M<axis><direction><steps>\n"
Example: "M1+500\n"  // Move axis 1 forward 500 steps
```

### G-code Style (Hackaday project)

```
Format: "X<x> Y<y> Z<z> A<angle>\n"
Example: "X100 Y50 Z200 A45\n"
```

### Binary Protocol (Professional)

- Fixed-size packets
- Checksums for reliability
- Higher throughput

### Recommendation:

Start with simple ASCII serial, upgrade to binary if needed.

---

## Power Supply Considerations

### From Projects:

- **amiravni:** 12V for motors via L293D
- **ethanleep:** 14V DC main supply
- **aamitn:** PID control with 12V nominal
- **Manual specs:** Motors rated 12V, 400mA typical

### Voltage Options:

1. **12V regulated:** Safest, matches motor rating
2. **14V-15V unregulated:** More torque, original spec
3. **Variable (PWM):** Best control, requires good drivers

### Your L298N Boards:

- Rated up to 46V input
- 2A per channel
- Built-in 5V regulator
- **Recommendation:** Use 12V supply, PWM for speed control

---

## Arduino Mega Capabilities

### Confirmed Working Projects:

- amiravni: Full 6-axis control + drawing
- ethanleep: Full control with distributed architecture
- Hackaday: "underpowered" for kinematics but worked

### Pin Availability Check:

- **PWM outputs needed:** 6 (ENA/ENB for 3x L298N) ✓ Have 15 PWM pins
- **Direction outputs:** 12 (IN1-4 for 3x L298N) ✓ Plenty of digital pins
- **Encoder inputs:** 12 (P0/P1 for 6 motors) ✓ Have 6 hardware interrupts + many digital
- **Microswitch inputs:** 5 ✓
- **Serial comm:** 1 (plus 3 additional Serial ports) ✓

### Conclusion:

Arduino Mega is MORE than capable. Timer interrupts + hardware interrupts sufficient.

---

## Architecture Recommendations

### Phase 1: Basic Motor Control

```
Arduino Mega → L298N boards → Motors
         ↓
    Read encoders (interrupts)
    Read microswitches
    Serial control from PC
```

### Phase 2: Add Positioning

```
+ Implement home search
+ Position tracking with encoders
+ Go-to-position commands
+ Soft limits for safety
```

### Phase 3: Add Intelligence

```
+ PID control loops
+ Speed ramping (acceleration/deceleration)
+ Coordinated multi-axis moves
+ Trajectory planning (if needed)
```

### Phase 4: Advanced (Optional)

```
+ Kinematics (forward/inverse)
+ Path planning
+ Vision integration
+ ROS interface
```

---

## Critical Code Patterns

### Encoder Reading (from amiravni pattern):

```cpp
volatile long encoderCount[6] = {0};

void setupEncoders() {
    // Set up timer for fast polling
    TCCR1B = 0;  // Stop timer
    TCNT1  = 0;  // Reset counter
    TCCR1B |= (1 << CS10); // No prescaling = ~16MHz
    TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt
}

ISR(TIMER1_OVF_vect) {
    for(int i=0; i<6; i++) {
        byte p0 = digitalRead(encoder_P0[i]);
        byte p1 = digitalRead(encoder_P1[i]);

        if(p0 != lastP0[i]) {
            if(p0 == p1) encoderCount[i]++;
            else encoderCount[i]--;
            lastP0[i] = p0;
        }
    }
}
```

### Homing Routine Pattern:

```cpp
void homeAxis(int axis) {
    // Move backward slowly
    setMotorSpeed(axis, -HOMING_SPEED);

    // Wait for microswitch
    while(!digitalRead(limitSwitch[axis])) {
        delay(10);
    }

    // Stop motor
    setMotorSpeed(axis, 0);

    // Zero encoder
    encoderCount[axis] = 0;

    // Back off switch
    moveToPosition(axis, BACKOFF_DISTANCE);
}
```

### Position Control Pattern:

```cpp
void moveToPosition(int axis, long targetPos) {
    long error = targetPos - encoderCount[axis];

    while(abs(error) > POSITION_TOLERANCE) {
        error = targetPos - encoderCount[axis];

        // Simple proportional control
        int speed = constrain(error * Kp, -MAX_SPEED, MAX_SPEED);
        setMotorSpeed(axis, speed);

        delay(10);
    }

    setMotorSpeed(axis, 0);
}
```

---

## Lessons from Failed Approaches

### Don't Do:

1. **Skip encoder filtering** - Noise causes position drift
2. **No home routine** - Impossible to recover from errors
3. **Ignore motor stall detection** - Can damage mechanics
4. **Run motors continuously at full voltage** - Overheating
5. **No emergency stop** - Safety critical
6. **Complex kinematics on Mega** - Slow, use PC instead

### Do:

1. **Test motors individually first**
2. **Test encoders separately**
3. **Add serial debug output**
4. **Implement soft limits**
5. **Start without PID, add later**
6. **Keep kinematics on PC/Python**
