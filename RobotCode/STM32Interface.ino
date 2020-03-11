#include "STM32Interface.h"

long STM32Interface::leftEncoder;
long STM32Interface::rightEncoder;

STM32Interface::STM32Interface() {
  if (!initialized) {
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
    pinMode(PIN_DISTANCE_SENSOR_FRONT, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), STM32Interface::encoderLeftChangeA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), STM32Interface::encoderLeftChangeB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), STM32Interface::encoderRightChangeA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), STM32Interface::encoderRightChangeB, CHANGE);
  }

  initialized = true;
}

STM32Interface::~STM32Interface() { }

void STM32Interface::setMotorOutput(float left, float right) {
  left = min(max(left, -1), 1);
  right = min(max(right, -1), 1);
  float leftSign = (left > 0) - (left < 0);
  float rightSign = (right > 0) - (right < 0);
  if (left == 0) { leftSign = 0; }
  if (right == 0) { rightSign = 0; }
  
  int left_motor_output = (int) abs(round(left * 100 + leftSign * 18));
  int right_motor_output = (int) abs(round(right * 100 + rightSign * 18));

  if (left_motor_output > 0) {
    if (left >= 0) {
      digitalWrite(PIN_MOTOR_LEFT_A, HIGH);
      digitalWrite(PIN_MOTOR_LEFT_B, LOW);
    } else {
      digitalWrite(PIN_MOTOR_LEFT_A, LOW);
      digitalWrite(PIN_MOTOR_LEFT_B, HIGH);
    }
  } else {
    digitalWrite(PIN_MOTOR_LEFT_A, LOW);
    digitalWrite(PIN_MOTOR_LEFT_B, LOW);
  }

  if (right_motor_output > 0) {
    if (right >= 0) {
      digitalWrite(PIN_MOTOR_RIGHT_A, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_B, LOW);
    } else {
      digitalWrite(PIN_MOTOR_RIGHT_A, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_B, HIGH);
    }
  } else {
    digitalWrite(PIN_MOTOR_RIGHT_A, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_B, LOW);
  }

  analogWrite(PIN_MOTOR_LEFT_PWM, left_motor_output);
  analogWrite(PIN_MOTOR_RIGHT_PWM, right_motor_output);
}

long STM32Interface::getLeftEncoderRaw() {
  return leftEncoder;
}

long STM32Interface::getRightEncoderRaw() {
  return rightEncoder;
}

float STM32Interface::getFrontLeftDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_FRONT_LEFT);
}

float STM32Interface::getFrontRightDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_FRONT_RIGHT);
}

float STM32Interface::getRearLeftDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_REAR_LEFT);
}

float STM32Interface::getRearRightDistance() {
  return analogRead(PIN_DISTANCE_SENSOR_REAR_RIGHT);
}

float STM32Interface::getRobotRadius() {
  return 4.8;
}

float STM32Interface::getWheelRadius() {
  return 2.15;
}

float STM32Interface::getTicksPerRevolution() {
  return 900;
}

PIDConstants STM32Interface::getLefttWheelPIDConstants() {
  return leftWheelPID;
}
  
PIDConstants STM32Interface::getRightWheelPIDConstants() {
  return rightWheelPID;
}

void STM32Interface::encoderLeftChangeA() {
  leftEncoder -= (1 - 2*digitalRead(PIN_ENCODER_LEFT_A)) * (1 - 2*digitalRead(PIN_ENCODER_LEFT_B));
}

void STM32Interface::encoderLeftChangeB() {
  leftEncoder += (1 - 2*digitalRead(PIN_ENCODER_LEFT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_LEFT_A));
}

void STM32Interface::encoderRightChangeA() {
  rightEncoder += (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A)) * (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B));
}

void STM32Interface::encoderRightChangeB() {
  rightEncoder -= (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A));
}
