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
// GOAL QUEUE SYSTEM
// ============================================================================
// Allows sequencing goals with parallel execution within each GoalGroup

const int MAX_QUEUE_STEPS = 99;

struct QueuedGoal {
  int motorIndex;       // Which motor (0-5), or -1 if slot empty
  JointGoal goal;       // What goal (FIND_HOME, MOVE_TO, etc.)
  long targetPosition;  // For MOVE_TO goals
};

struct QueueGoalGroup {
  QueuedGoal goals[6];  // Up to 6 parallel goals per GoalGroup
  int goalCount;        // How many goals in this GoalGroup
  const char* name;     // Motor name for debugging
};

// Queue state
QueueGoalGroup goalQueue[MAX_QUEUE_STEPS];
int currentQueueGoalGroup = -1;  // -1 = queue not running, >= 0 = executing that GoalGroup
int totalQueueGoalGroups = 0;    // Total steps defined in queue
int buildingGoalGroup = -1;      // Which GoalGroup we're currently adding goals to
bool queueFaulted = false;       // True if queue stopped due to motor fault

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void setMotor(int ScorbotJointIndex, int speed, bool isDifferentialActive = true);
void stopMotor(int ScorbotJointIndex, bool isDifferentialActive = true);

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
// GOAL QUEUE FUNCTIONS
// ============================================================================

// Clear and reset the queue
inline void queueClear() {
  currentQueueGoalGroup = -1;
  totalQueueGoalGroups = 0;
  buildingGoalGroup = -1;
  queueFaulted = false;

  // Zero out all queue data
  for (int s = 0; s < MAX_QUEUE_STEPS; s++) {
    goalQueue[s].goalCount = 0;
    for (int g = 0; g < 6; g++) {
      goalQueue[s].goals[g].motorIndex = -1;
      goalQueue[s].goals[g].goal = GOAL_IDLE;
      goalQueue[s].goals[g].targetPosition = 0;
    }
  }

  // Serial.println("Queue: Cleared");
}

// Begin defining a new group (call before adding goals to that GoalGroup)
inline void queueCreateGoalGroup(const char* groupName = "") {
  if (totalQueueGoalGroups >= MAX_QUEUE_STEPS) {
    Serial.println("Queue: ERROR - max steps reached!");
    return;
  }

  buildingGoalGroup = totalQueueGoalGroups;
  totalQueueGoalGroups++;
  goalQueue[buildingGoalGroup].goalCount = 0;
  goalQueue[buildingGoalGroup].name = groupName;

  // Serial.print("Queue: Creating GoalGroup ");
  // Serial.print(buildingGoalGroup);
  // if (groupName[0] != '\0') {
  //   Serial.print(" (");
  //   Serial.print(groupName);
  //   Serial.print(")");
  // }
  Serial.println();
}

// Add a goal to the current GoalGroup being built
inline void queueAddGoal(int motorIndex, JointGoal goal, long targetPosition = 0) {
  if (buildingGoalGroup < 0 || buildingGoalGroup >= MAX_QUEUE_STEPS) {
    Serial.println("Queue: ERROR - call queueCreateGoalGroup() first!");
    return;
  }

  int count = goalQueue[buildingGoalGroup].goalCount;
  if (count >= 6) {
    Serial.println("Queue: ERROR - max 6 goals per GoalGroup!");
    return;
  }

  goalQueue[buildingGoalGroup].goals[count].motorIndex = motorIndex;
  goalQueue[buildingGoalGroup].goals[count].goal = goal;
  goalQueue[buildingGoalGroup].goals[count].targetPosition = targetPosition;
  goalQueue[buildingGoalGroup].goalCount++;

  // Serial.print("Queue:   Added ");
  // Serial.print(SCORBOT_REF[motorIndex].name);
  // Serial.print(" -> ");
  // Serial.println(MOTOR_GOAL_NAMES[goal]);
}

// Dispatch all goals for a given GoalGroup
inline void queueDispatchStep(int stepIndex) {
  if (stepIndex < 0 || stepIndex >= totalQueueGoalGroups)
    return;

  QueueGoalGroup* GoalGroup = &goalQueue[stepIndex];

  // Serial.print("Queue: Starting GoalGroup ");
  // Serial.print(stepIndex);
  // if (GoalGroup->name != nullptr && GoalGroup->name[0] != '\0') {
  //   Serial.print(" (");
  //   Serial.print(GoalGroup->name);
  //   Serial.print(")");
  // }
  // Serial.print(" with ");
  // Serial.print(GoalGroup->goalCount);
  // Serial.println(" goal(s)");

  for (int g = 0; g < GoalGroup->goalCount; g++) {
    QueuedGoal* qg = &GoalGroup->goals[g];
    if (qg->motorIndex >= 0 && qg->motorIndex < ScorbotJointIndex_COUNT) {
      // For MOVE_TO goals, set the target first
      if (qg->goal == GOAL_MOVE_TO) {
        jointState[qg->motorIndex].targetEncoderCountToMoveTo = qg->targetPosition;
      }
      setGoal(qg->motorIndex, qg->goal);
    }
  }
}

// Start executing the queue from GoalGroup 0
inline void queueStart() {
  if (totalQueueGoalGroups == 0) {
    Serial.println("Queue: ERROR - no steps defined!");
    return;
  }

  queueFaulted = false;
  currentQueueGoalGroup = 0;

  // Serial.print("Queue: Starting execution with ");
  // Serial.print(totalQueueGoalGroups);
  // Serial.println(" GoalGroup(s)");

  queueDispatchStep(0);
}

// Check if queue is currently running
inline bool queueIsRunning() {
  return currentQueueGoalGroup >= 0 && !queueFaulted;
}

// Check if queue stopped due to fault
inline bool queueIsFaulted() {
  return queueFaulted;
}

// Check if current GoalGroup is complete and advance to next GoalGroup if so
// Call this in the main loop
inline void queueAdvanceIfReady() {
  if (currentQueueGoalGroup < 0 || queueFaulted)
    return;  // Queue not running

  QueueGoalGroup* GoalGroup = &goalQueue[currentQueueGoalGroup];
  bool allComplete = true;

  // Check all goals in current GoalGroup
  for (int g = 0; g < GoalGroup->goalCount; g++) {
    int motorIdx = GoalGroup->goals[g].motorIndex;
    if (motorIdx < 0 || motorIdx >= ScorbotJointIndex_COUNT)
      continue;

    JointGoal state = jointState[motorIdx].currentGoal;

    // Check for fault
    if (state == GOAL_FAULT) {
      Serial.print("Queue: FAULT on ");
      Serial.print(SCORBOT_REF[motorIdx].name);
      Serial.print(" at GoalGroup ");
      Serial.print(currentQueueGoalGroup);
      Serial.println(" - queue halted!");
      queueFaulted = true;
      return;
    }

    // Check if still running (not IDLE)
    if (state != GOAL_IDLE) {
      allComplete = false;
    }
  }

  if (!allComplete)
    return;  // Still waiting for goals to finish

  // All goals in current GoalGroup complete!
  // Serial.print("Queue: GoalGroup ");
  // Serial.print(currentQueueGoalGroup);
  // Serial.println(" complete");

  // Advance to next GoalGroup
  currentQueueGoalGroup++;

  if (currentQueueGoalGroup >= totalQueueGoalGroups) {
    // Queue finished!
    Serial.println("Queue: All steps complete");
    currentQueueGoalGroup = -1;  // Mark queue as not running
    return;
  }

  // Dispatch next GoalGroup
  queueDispatchStep(currentQueueGoalGroup);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(500);  // Brief delay to allow Serial to initialize
  // Clear any garbage data in the buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("\n SCORBOT START ===========================");

  // Initialize hardware
  setupAllPins();
  initializeAllJointStates();

  // Safety: stop all motors
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    // stopMotor(i);
  }

  // Initialize encoder check time
  for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
    jointState[i].lastEncoderCheckTime = millis();
  }

  // Build homing sequence using the goal queue
  // GoalGroups execute sequentially, goals within each group run in parallel
  queueClear();

  queueCreateGoalGroup("base & shoulder find home");
  queueAddGoal(MOTOR_BASE, GOAL_FIND_HOME);
  queueAddGoal(MOTOR_SHOULDER, GOAL_FIND_HOME);

  queueCreateGoalGroup("elbow find home");
  queueAddGoal(MOTOR_ELBOW, GOAL_FIND_HOME);

  queueCreateGoalGroup("wrist roll find home");
  queueAddGoal(MOTOR_WRIST_ROLL, GOAL_FIND_HOME);

  queueCreateGoalGroup("wrist pitch find home");
  queueAddGoal(MOTOR_WRIST_PITCH, GOAL_FIND_HOME);

  queueCreateGoalGroup("gripper find home");
  queueAddGoal(MOTOR_GRIPPER, GOAL_FIND_HOME);

  queueStart();  // Begin executing the queue
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Emergency stop check - first thing in loop
  if (digitalRead(ESTOP_PIN) == LOW) {
    for (int i = 0; i < ScorbotJointIndex_COUNT; i++) {
      stopMotor(i);
      jointState[i].currentGoal = GOAL_FAULT;
    }
    // Also halt the queue on E-stop
    if (queueIsRunning()) {
      queueFaulted = true;
      Serial.println("Queue: Halted by E-STOP");
    }
    Serial.println("EMERGENCY STOP");
    while (digitalRead(ESTOP_PIN) == LOW) {
      delay(10);  // Wait for button release
    }
    Serial.println("E-Stop released - reset to continue");
    return;  // Skip rest of loop until reset
  }

  updateAllEncoders();  // CRITICAL: Update all encoders every loop

  checkAllStalls();

  checkAllHomeSwitches();

  // Update all active goals
  doAllGoals();

  // Check if queue GoalGroup is complete and advance
  queueAdvanceIfReady();

  delay(1);
}

// ============================================================================
// GOALS
// ============================================================================

// Set goal for specific motor
inline void setGoal(int ScorbotJointIndex, JointGoal goal) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  // Reset goal-specific flags
  jointState[ScorbotJointIndex].goalStartTime = millis();
  resetStallDetection(ScorbotJointIndex);
  jointState[ScorbotJointIndex].totalStallsThisGoal = 0;  // Reset stall count for new goal

  // Reset wrist motor speeds when entering homing mode
  if (goal == GOAL_FIND_HOME &&
      (ScorbotJointIndex == MOTOR_WRIST_PITCH || ScorbotJointIndex == MOTOR_WRIST_ROLL)) {
    jointState[MOTOR_WRIST_PITCH].motorSpeed = 0;
    jointState[MOTOR_WRIST_ROLL].motorSpeed = 0;
  }

  // CRITICAL: Set the goal BEFORE executing motor commands
  // This ensures bypass checks see the correct goal state
  jointState[ScorbotJointIndex].currentGoal = goal;

  switch (goal) {
    case GOAL_FIND_HOME:
      jointState[ScorbotJointIndex].hasFoundHome = false;
      setMotor(ScorbotJointIndex, 80);  // % power for that motor, positive is clockwise
      break;

      // case GOAL_GO_HOME:
      // setGoalGoHome(ScorbotJointIndex);
      //   break;

      // case GOAL_CALIBRATE_RANGE_CCW:
      //   setsGoalCalibrateRange_CCW(ScorbotJointIndex);
      //   break;

      // case GOAL_MOVE_TO:
      //   setsGoalMoveTo(ScorbotJointIndex);
      //   break;

    case GOAL_IDLE:
      stopMotor(ScorbotJointIndex);  // stop motor when goal set to idle for double safety
      break;
    case GOAL_FAULT:
      stopMotor(ScorbotJointIndex);  // stop motor when goal set to idle for double safety
      break;
  }
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
// ------------------------------------------------------------------------
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
    Serial.println("is home");
    return;
  }

  // Otherwise, keep searching
  // Motor is already moving (started in setGoal)
  // Stall detection will reverse direction if needed
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// ------------------------------------------------------------------------

inline void setMotor(int ScorbotJointIndex, int speed, bool isDifferentialActive = true) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;
  if (speed < -99 || speed > 99)
    return;

  // Store the abstract/logical speed in jointState
  jointState[ScorbotJointIndex].motorSpeed = speed;

  // Serial.print("setMotor ");
  // Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
  // Serial.print(", ");
  // Serial.println(speed);

  // DIFFERENTIAL DRIVE FOR WRIST
  // For wrist pitch/roll, differential control is needed because the physical
  // motors are coupled - moving one motor alone produces combined pitch+roll
  // motion. The differential math produces pure pitch or pure roll motion.
  if (isDifferentialActive &&
      (ScorbotJointIndex == MOTOR_WRIST_PITCH || ScorbotJointIndex == MOTOR_WRIST_ROLL)) {
    // Apply differential motor commands to both physical motors
    // Reads abstract speeds from jointState and calculates physical motor outputs
    int motor4Speed, motor5Speed;

    // Differential math based on mechanical coupling
    // motors Same direction = pitch, Opposite direction = roll
    // old code for reference and debugging
    // // makes wrist pitch up only
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CCW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CCW_pin, LOW);

    // //  wrist pitch down,
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CCW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CCW_pin, HIGH);

    // wrist pitch nothing , clockwise roll
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CCW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CCW_pin, LOW);

    // wrist pitch nothing  counter clockwise roll
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CW_pin, HIGH);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_PITCH].CCW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CW_pin, LOW);
    // digitalWrite(SCORBOT_REF[MOTOR_WRIST_ROLL].CCW_pin, HIGH);

    int raw_motor4 = jointState[MOTOR_WRIST_PITCH].motorSpeed -
                     jointState[MOTOR_WRIST_ROLL].motorSpeed;  // Pitch motor (physical motor 4)
    int raw_motor5 = jointState[MOTOR_WRIST_PITCH].motorSpeed +
                     jointState[MOTOR_WRIST_ROLL].motorSpeed;  // Roll motor (physical motor 5)

    // Proportional scaling to maintain motion ratio
    int maxAbsValue = max(abs(raw_motor4), abs(raw_motor5));
    if (maxAbsValue > 99) {
      float scaleFactor = 99.0 / maxAbsValue;
      raw_motor4 = (int)(raw_motor4 * scaleFactor);
      raw_motor5 = (int)(raw_motor5 * scaleFactor);
    }

    // Clamp to valid range (safety)
    motor4Speed = constrain(raw_motor4, -99, 99);
    motor5Speed = constrain(raw_motor5, -99, 99);

    // Apply to physical motors using direct control (bypasses differential logic)
    setMotorDirect(MOTOR_WRIST_PITCH, motor4Speed);
    setMotorDirect(MOTOR_WRIST_ROLL, motor5Speed);

    return;
  }

  // NORMAL (NON-DIFFERENTIAL) MOTOR CONTROL
  setMotorDirect(ScorbotJointIndex, speed);
}

// ------------------------------------------------------------------------
// Direct motor control - sets physical motor without differential interception
// Used internally by differential system to avoid recursion
inline void setMotorDirect(int ScorbotJointIndex, int speed) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;
  if (speed < -99 || speed > 99)
    return;

  // Debug output for wrist motors
  // if (ScorbotJointIndex == MOTOR_WRIST_PITCH || ScorbotJointIndex == MOTOR_WRIST_ROLL) {
  //   Serial.print("setMotor Direct ");
  //   Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
  //   Serial.print(", ");
  //   Serial.print(speed);
  //   Serial.println();
  // }

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
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);
    digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
    analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, 0);
    // Don't modify jointState.motorSpeed here - that's the logical speed managed by setMotor()
    return;
  }

  // Don't modify jointState.motorSpeed here - that's the logical speed managed by setMotor()
  pwmValue = map(abs(speed), 0, 99, motor_min, 255);
  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, pwmValue);
}

// ------------------------------------------------------------------------
// Stop a motor
// Usage: stopMotor(MOTOR_BASE);
inline void stopMotor(int ScorbotJointIndex, bool isDifferentialActive = true) {
  // Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
  // Serial.println(" stopMotor");

  setMotor(ScorbotJointIndex, 0, isDifferentialActive);
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
// Reset stall detection state - call when starting motor or changing direction
inline void resetStallDetection(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return;

  jointState[ScorbotJointIndex].stallCounter = 0;
  jointState[ScorbotJointIndex].lastEncoderCount = jointState[ScorbotJointIndex].encoderCount;
  jointState[ScorbotJointIndex].lastEncoderCheckTime = millis();
}

// ------------------------------------------------------------------------
inline bool checkStall(int ScorbotJointIndex) {
  if (ScorbotJointIndex < 0 || ScorbotJointIndex >= ScorbotJointIndex_COUNT)
    return false;
  if (jointState[ScorbotJointIndex].motorSpeed == 0)
    return false;

  if (jointState[ScorbotJointIndex].currentGoal == GOAL_IDLE)
    return false;  // MOTOR_WRIST_PITCH and  MOTOR_WRIST_ROLL drive each other, so ignore stalls on
                   // the idle joint

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
    STALL_MIN_ENCODER_CHANGE = 1;  // Only need 1 encoder change (instead of 2)
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
        if (jointState[ScorbotJointIndex].motorSpeed == 0) {
          // motor is idle, something is weird, set goal to IDLE
          jointState[ScorbotJointIndex].currentGoal = GOAL_IDLE;
        } else {
          Serial.print(SCORBOT_REF[ScorbotJointIndex].name);

          // stalled but we don't know which side, if either, it is stalled at,
          // so use motor direction
          if (jointState[ScorbotJointIndex].motorSpeed > 0) {
            Serial.println(": stalled going clockwise, reversing");

          } else if (jointState[ScorbotJointIndex].motorSpeed < 0) {
            // stalled when moving clockwise, we don't know which side, if either, it is stalled at,
            // so assume at end of CW rotation

            Serial.println(": stalled going counter clockwise, reversing");
          }

          int reversedSpeed = jointState[ScorbotJointIndex].motorSpeed * -1;
          if (ScorbotJointIndex == MOTOR_WRIST_PITCH || ScorbotJointIndex == MOTOR_WRIST_ROLL) {
            // Clear the other wrist axis to avoid differential interference
            setMotor(ScorbotJointIndex == MOTOR_WRIST_PITCH ? MOTOR_WRIST_ROLL : MOTOR_WRIST_PITCH,
                     0);
          }
          setMotor(ScorbotJointIndex, reversedSpeed);
          resetStallDetection(ScorbotJointIndex);
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
      // Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
      // Serial.println(": hit switch going clockwise, reset home position");
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
      // Serial.print(SCORBOT_REF[ScorbotJointIndex].name);
      // Serial.println(": off switch going counter clockwise, reset home position");
      // future task: validate how many steps difference from CW rising edge to CCW falling edge

      jointState[ScorbotJointIndex].lastHomeSwitchDebounceTime = MillisSinceProgramStart;
      jointState[ScorbotJointIndex].encoderCount = 0;  // Set CW edge as zero
      jointState[ScorbotJointIndex].hasFoundHome = true;
      return true;
    }
  }

  return false;
}

// ------------------------------------------------------------------------

inline void checkAllHomeSwitches() {
  for (int ScorbotJointIndex = 0; ScorbotJointIndex < ScorbotJointIndex_COUNT;
       ScorbotJointIndex++) {
    if (checkIfHomeSwitchEdge(ScorbotJointIndex)) {
      // deal with the stall with business logic
    }
  }
  return;
}

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
