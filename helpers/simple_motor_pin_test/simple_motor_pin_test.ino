#include "../../scorbot.h"

int ScorbotJointIndex = 5;
int speed = 50;

void setup() {
  Serial.begin(115200);
  delay(500);

  //   analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, 200);

  //   digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
  //   digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, HIGH);
  //   delay(1000);
}

void loop() {
  int motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CW;

  int pwmValue = map(abs(speed), 0, 99, motor_min, 255);
  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, pwmValue);

  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, HIGH);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);

  delay(1000);

  motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CCW;
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);

  delay(2000);

  // CCW is open

  motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CCW;

  pwmValue = map(abs(speed), 0, 99, motor_min, 255);
  analogWrite(SCORBOT_REF[ScorbotJointIndex].pwm_pin, pwmValue);

  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, HIGH);

  delay(1000);

  motor_min = SCORBOT_REF[ScorbotJointIndex].motor_min_CCW;
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CW_pin, LOW);
  digitalWrite(SCORBOT_REF[ScorbotJointIndex].CCW_pin, LOW);

  delay(10000);
}