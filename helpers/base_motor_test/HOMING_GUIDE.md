# SCORBOT Base Motor - Homing System User Guide

## Overview

This firmware implements a complete homing system for the Scorbot base motor with:
- **Encoder-based position tracking** - Tracks position relative to home
- **Microswitch home detection** - Establishes zero reference position
- **Stall detection** - Prevents damage by detecting when motor hits obstacles
- **Software limits** - Prevents motion beyond safe range after homing
- **Bidirectional search** - Finds home switch from any starting position

## How It Works

### The Homing Problem (Answered)

**Q: Why do we need homing?**
A: When the Arduino resets, it loses all position information. The encoder only tracks *relative* motion, not absolute position. Homing establishes a known reference point.

**Q: What if the motor hits the endpoint before finding the switch?**
A: The system uses **stall detection**:
- Every 50ms, it checks if the encoder position changed
- If the motor is powered but encoder hasn't moved for 100ms → **stall detected**
- On stall: stops immediately, reverses direction, tries the other way
- If stalled in both directions without finding switch → **FAULT state**

**Q: Where is the microswitch located?**
A: The home switch should be at the **CENTER** of the base rotation range (±155° from center, 310° total range).

### Homing Sequence

```
1. UNINITIALIZED (power-on)
   ↓
2. HOMING_SEARCH_CCW (search counter-clockwise at 20% speed)
   ↓ (switch found) OR ↓ (stall detected)
   │                    └→ 3. HOMING_SEARCH_CW (try clockwise)
   ↓
4. HOMING_BACKOFF (move away 50 encoder counts)
   ↓
5. HOMING_APPROACH (slow approach at 12% speed)
   ↓ (switch triggered)
6. HOMED (position = 0, ready for operation!)
```

If the switch is found in either direction, it backs off slightly, then re-approaches slowly for maximum accuracy. This ensures repeatable positioning within ±5 encoder counts.

## Installation

### Hardware Setup

1. **Motor connections (L298N Board 1):**
   - Arduino pin 22 → L298N IN1 (DIR1)
   - Arduino pin 23 → L298N IN2 (DIR2)
   - Arduino pin 2 → L298N ENA (PWM speed control)
   - L298N Motor A terminals → Base motor
   - 12V power supply → L298N power terminals
   - Arduino GND → L298N GND

2. **Encoder connections:**
   - Encoder P0 → Arduino pin 18 (interrupt capable)
   - Encoder P1 → Arduino pin 19 (interrupt capable)
   - Encoder +5V → Arduino 5V (through 47Ω resistor if using LED)
   - Encoder GND → Arduino GND

3. **Microswitch:**
   - Base home switch → Arduino pin 46
   - Other side of switch → Arduino GND
   - (Uses internal INPUT_PULLUP, active LOW)

### Upload Firmware

1. Open `base_motor_test.ino` in Arduino IDE
2. Select **Board**: Arduino Mega 2560
3. Select correct COM port
4. Click Upload
5. Open Serial Monitor (9600 baud)

## Usage

### First Time Setup

1. **Power on** - Serial monitor should show:
   ```
   ========================================
   SCORBOT BASE MOTOR - HOMING SYSTEM
   ========================================
   [OK] Motor pins configured
   [OK] Encoder configured
   [OK] Home switch configured

   READY - Awaiting homing command
   Send 'h' to start homing sequence
   ========================================
   ```

2. **Start homing** - Send `h` command via Serial Monitor:
   - Motor will search slowly for the home switch
   - Watch the arm move - it should find the switch without hitting hard stops
   - If it stalls before finding the switch, it automatically reverses
   - When found, it backs off and re-approaches slowly
   - Finally prints: **"HOMING COMPLETE!"**

3. **Check position** - Send `p` to see status:
   ```
   --- STATUS ---
   State: HOMED
   Position: 0 counts
   Limits: [-1500 to 1500]
   Position in range: 50.0%
   Switch: not pressed
   Motor: stopped
   --------------
   ```

### Normal Operation Commands

Once homed, you can control the motor with these serial commands:

| Command | Action |
|---------|--------|
| `h` | Start/restart homing sequence |
| `+` | Move clockwise (hold key to keep moving) |
| `-` | Move counter-clockwise |
| `s` | Stop motor immediately |
| `p` | Print current status (position, state, limits) |
| `r` | Reset to unhomed state (for testing) |

**Note:** The motor will only move after successful homing. If you try `+` or `-` before homing, it will display an error.

### Testing Procedure

#### Test 1: Basic Homing
1. Power on, send `h`
2. Observe: searches → finds switch → backs off → slow approach → "HOMING COMPLETE!"
3. Verify: send `p`, position should be at 0

#### Test 2: Homing from Different Positions
1. After homing, manually move the arm to various positions
2. Send `r` to reset to unhomed state
3. Send `h` to home again
4. Repeat 10 times from different starting positions
5. Check: each time should return to position 0 (within ±5 counts)

#### Test 3: Stall Detection
1. After homing, send `+` or `-` to move motor
2. **Manually hold the arm** to prevent motion
3. Within 100-150ms, should print: **"!!! STALL DETECTED !!!"**
4. Motor stops immediately
5. This protects against mechanical damage

#### Test 4: Software Limits
1. After homing, send `+` repeatedly to move toward CW limit
2. Watch encoder count increase (use `p` to check)
3. At count ~1500, should stop and print: **"!!! SOFTWARE LIMIT REACHED !!!"**
4. Same test for CCW direction with `-` (stops at ~-1500)

#### Test 5: Bidirectional Search
1. Manually position arm near CCW hard stop (left of home switch)
2. Power cycle or send `r` then `h`
3. Motor searches CCW, stalls, then reverses
4. Finds home switch from CW direction
5. Successfully homes

### Calibrating Software Limits

The default limits (±1500 counts) are conservative estimates. To calibrate:

1. **Find actual CW limit:**
   - Home the motor (`h`)
   - Slowly move CW using `+` command
   - Watch position with `p` command
   - **Manually stop** just before hitting mechanical limit
   - Record the encoder count (e.g., 1847)

2. **Find actual CCW limit:**
   - From home, slowly move CCW using `-`
   - Stop before mechanical limit
   - Record count (e.g., -1823)

3. **Update firmware:**
   - Edit lines 72-73 in `base_motor_test.ino`:
     ```arduino
     #define MAX_POSITION_CW 1800      // Your measured value - 50
     #define MAX_POSITION_CCW -1800    // Your measured value + 50
     ```
   - Leave 50-count safety margin to avoid hitting hard stops
   - Re-upload firmware

## Tuning Parameters

If homing isn't working well, you can adjust these parameters (lines 53-74):

### Homing Speeds
```arduino
#define HOMING_SEARCH_SPEED 50      // Initial search speed (20% of max)
#define HOMING_APPROACH_SPEED 30     // Final approach speed (12% of max)
```
- Increase if homing is too slow
- Decrease if overshooting the switch

### Stall Detection
```arduino
#define STALL_CHECK_INTERVAL_MS 50   // How often to check encoder
#define STALL_THRESHOLD_MS 100       // Time without motion = stall
#define STALL_MIN_ENCODER_CHANGE 2   // Min encoder counts expected
```
- If false stalls during slow motion: decrease `STALL_MIN_ENCODER_CHANGE` to 1
- If not detecting real stalls: increase `STALL_THRESHOLD_MS` to 150
- For faster response: decrease `STALL_CHECK_INTERVAL_MS` to 30

### Homing Accuracy
```arduino
#define HOMING_BACKOFF_COUNTS 50     // Distance to back off from switch
```
- Increase if switch is chattering/bouncing
- Decrease for faster homing (but may reduce accuracy)

## Troubleshooting

### "FAULT: Stalled in both directions"
**Cause:** Switch not detected, hit hard stops in both directions
- Check microswitch wiring (pin 46 to GND when pressed)
- Test switch: send `p` command, manually press switch, send `p` again - should show "PRESSED"
- Check switch alignment - arm might not be triggering it
- Check encoder wiring - if encoder not working, stall detection won't work

### Homing completes but position drifts during motion
**Cause:** Encoder not reading reliably
- Check encoder wiring (pins 18, 19)
- Check encoder power (5V, GND)
- Test: manually rotate motor slowly, send `p` repeatedly - count should change
- May need to add pull-up resistors on encoder lines

### Stall detection too sensitive
**Symptom:** False stalls during normal operation
- Increase `STALL_THRESHOLD_MS` from 100 to 150
- Decrease `STALL_MIN_ENCODER_CHANGE` from 2 to 1
- Check motor power supply - low voltage causes slow motion

### Motor doesn't move during homing
**Cause:** Motor driver not responding
- Check L298N connections (pins 22, 23, 2)
- Check 12V power supply to L298N
- Check motor connections to L298N Motor A terminals
- Try simple test: manually call `moveMotorCW(100)` in code

### Home position not repeatable
**Symptom:** Position varies by >10 counts between homing cycles
- Slow down `HOMING_APPROACH_SPEED` (try 20 instead of 30)
- Increase `HOMING_BACKOFF_COUNTS` (try 100 instead of 50)
- Add debouncing to switch (already has 10ms, try increasing)
- Check for mechanical play in switch mounting

## Safety Features

This firmware includes multiple layers of safety:

1. **Stall Detection** - Stops motor within 100ms if encoder stops moving
2. **Software Limits** - Prevents motion beyond calibrated range
3. **Timeout Protection** - Aborts homing if taking >10 seconds
4. **Switch Debouncing** - Prevents false triggers from electrical noise
5. **Velocity Limiting** - Automatically slows near limits
6. **Emergency Stop** - `s` command stops motor immediately

## Next Steps

After successfully homing the base motor:

1. **Calibrate limits** - Measure actual range, update MAX_POSITION constants
2. **Record encoder resolution** - Count encoder ticks for full 310° rotation
3. **Implement position control** - Add PID or step-based positioning
4. **Expand to other axes** - Use this code as template for shoulder, elbow, wrist
5. **Add serial protocol** - Create command structure for external control (PC, ROS, etc.)
6. **Multi-axis homing** - Coordinate homing sequence for all 6 motors

## Technical Details

### Encoder Resolution
- The base motor encoder resolution depends on your specific encoder
- Typical values: 500-2000 counts per revolution
- Measure by: `counts_per_360° = (encoderCount_at_CW_limit - encoderCount_at_CCW_limit) / 310° * 360°`

### State Machine
The firmware uses a robust state machine to handle the homing sequence. See code comments for detailed flow.

### Interrupt-Based Encoder Reading
- Uses Arduino hardware interrupts for accurate encoder tracking
- Both encoder channels trigger interrupts on CHANGE (rising and falling edges)
- Quadrature decoding algorithm determines direction from state transitions
- Works even at high motor speeds

### Memory Usage
- Program: ~12KB (Mega 2560 has 256KB available)
- RAM: ~400 bytes (Mega 2560 has 8KB available)
- Plenty of room for expansion to 6 motors

## Support

For issues or questions:
- Check the troubleshooting section above
- Review code comments in `base_motor_test.ino`
- Test individual components (motor, encoder, switch) separately
- Refer to SCORBOT documentation for mechanical details
