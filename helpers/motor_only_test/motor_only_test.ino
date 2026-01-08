/*
  MOTOR ONLY TEST - Check if motor spins without encoder checking

  This will spin the motor without any encoder or stall detection.
  Watch if the motor actually rotates.

*/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#include "/Users/steveturbek/Documents/scorbot_controller/scorbot_controller/scorbot.h"

void setup() {
  Serial.begin(9600);

  // Setup the base motor pins
setupAllMotors();

  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("Motor Only Test");
  Serial.println("Motor should spin CW, stop, then CCW, stop");
  Serial.println();
}

void loop() {
//ScorbotJointIndex_COUNT
  for (int ScorbotJointNum = 0; ScorbotJointNum < ScorbotJointIndex_COUNT; ScorbotJointNum++) {

    Serial.println(SCORBOT[ScorbotJointNum].name);

    Serial.println("CW");
    moveMotor(ScorbotJointNum, 50, true);  
    delay(2000);

    stopMotor(ScorbotJointNum);
    delay(1000);

    Serial.println("CCW");
    moveMotor(ScorbotJointNum, 50, false);  
    delay(2000);

    Serial.println("");
    stopMotor(ScorbotJointNum);
    delay(1000);
    

  }
}
