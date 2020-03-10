#include "TeensyInterface.h"

#define MAX_VOLTAGE 60
#define MINIMUM_VOLTAGE 28
#define DESIRED_MAX_VOLTAGE (MAX_VOLTAGE - MINIMUM_VOLTAGE)
#define WHEEL_BASE_LENGTH 10.1 // centimeters
#define WHEEL_RADIUS 2.15 // centimeters
#define TICKS_PER_ROTATION 910

long TeensyInterface::leftEncoder;
long TeensyInterface::rightEncoder;

TeensyInterface::TeensyInterface() {
  pinMode(PIN_MOTOR_LEFT_A, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_B, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_A, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_B, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
  pinMode(PIN_ENCODER_RIGHT_A, INPUT);
  pinMode(PIN_ENCODER_RIGHT_B, INPUT);

  pinMode(PIN_DISTANCE_SENSOR_FRONT_LEFT, INPUT);
  pinMode(PIN_DISTANCE_SENSOR_FRONT_RIGHT, INPUT);
  pinMode(PIN_DISTANCE_SENSOR_REAR_LEFT, INPUT);
  pinMode(PIN_DISTANCE_SENSOR_REAR_RIGHT, INPUT);
  pinMode(PIN_DISTANCE_SENSOR_FRONT_FRONT, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), TeensyInterface::encoderLeftChangeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), TeensyInterface::encoderLeftChangeB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), TeensyInterface::encoderRightChangeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), TeensyInterface::encoderRightChangeB, CHANGE);
}

TeensyInterface::~TeensyInterface() {  }

void TeensyInterface::setMotorOutput(float left, float right) {
  left = min(max(left, -1), 1);
  right = min(max(right, -1), 1);
  float leftSign = (left > 0) - (left < 0);
  float rightSign = (right > 0) - (right < 0);
  int left_motor_output = (int) abs(round(left * DESIRED_MAX_VOLTAGE + MINIMUM_VOLTAGE * leftSign));
  int right_motor_output = (int) abs(round(right * DESIRED_MAX_VOLTAGE + MINIMUM_VOLTAGE * rightSign));

  if (left_motor_output > 0) {
    digitalWrite(PIN_MOTOR_LEFT_A, left >= 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_LEFT_B, left >= 0 ? LOW : HIGH);
  } else {
    digitalWrite(PIN_MOTOR_LEFT_A, LOW);
    digitalWrite(PIN_MOTOR_LEFT_B, LOW);
  }

  if (right_motor_output > 0) {
    digitalWrite(PIN_MOTOR_RIGHT_A, right >= 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_RIGHT_B, right >= 0 ? LOW : HIGH);
  } else {
    digitalWrite(PIN_MOTOR_RIGHT_A, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_B, LOW);
  }

  analogWrite(PIN_MOTOR_LEFT_PWM, left_motor_output);
  analogWrite(PIN_MOTOR_RIGHT_PWM, right_motor_output);
}

long TeensyInterface::getLeftEncoderRaw() {
  return leftEncoder;
}

long TeensyInterface::getRightEncoderRaw() {
  return rightEncoder;
}

int TeensyInterface::getFrontLeftDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_FRONT_LEFT);
}

int TeensyInterface::getFrontRightDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_FRONT_RIGHT);
}

int TeensyInterface::getRearLeftDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_REAR_LEFT);
}

int TeensyInterface::getRearRightDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_REAR_RIGHT);
}

int TeensyInterface::getFrontFrontDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_FRONT_FRONT);  
}

float TeensyInterface::getRobotRadius() {
  return WHEEL_BASE_LENGTH / 2;
}

float TeensyInterface::getWheelRadius() {
  return WHEEL_RADIUS;
}

float TeensyInterface::getTicksPerRevolution() {
  return TICKS_PER_ROTATION;
}

PIDConstants TeensyInterface::getLeftWheelPIDConstants() {
  return leftWheelPID;
}

PIDConstants TeensyInterface::getRightWheelPIDConstants() {
  return rightWheelPID;
}

void TeensyInterface::encoderLeftChangeA() {
  leftEncoder -= (1 - 2*digitalRead(PIN_ENCODER_LEFT_A)) * (1 - 2*digitalRead(PIN_ENCODER_LEFT_B));
}

void TeensyInterface::encoderLeftChangeB() {
  leftEncoder += (1 - 2*digitalRead(PIN_ENCODER_LEFT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_LEFT_A));
}

void TeensyInterface::encoderRightChangeA() {
  rightEncoder += (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A)) * (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B));
}

void TeensyInterface::encoderRightChangeB() {
  rightEncoder -= (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A));
}
