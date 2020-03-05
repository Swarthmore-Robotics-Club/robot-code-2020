#include "robot_include.h"

// state

long left_encoder;
long right_encoder;
long prev_left_encoder;
long prev_right_encoder;

float left_velocity;
float right_velocity;

int left_motor_output;
int right_motor_output;

// state //

void encoder_left_A_change() {
  left_encoder -= (1 - 2*digitalRead(PIN_ENCODER_LEFT_A)) * (1 - 2*digitalRead(PIN_ENCODER_LEFT_B));
}

void encoder_left_B_change() {
  left_encoder += (1 - 2*digitalRead(PIN_ENCODER_LEFT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_LEFT_A));
}

void encoder_right_A_change() {
  right_encoder += (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A)) * (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B));
}

void encoder_right_B_change() {
  right_encoder -= (1 - 2*digitalRead(PIN_ENCODER_RIGHT_B)) *  (1 - 2*digitalRead(PIN_ENCODER_RIGHT_A));
}

void setup_robot_interface() {
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

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), encoder_left_A_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), encoder_left_B_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), encoder_right_A_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), encoder_right_B_change, CHANGE);

  left_encoder = 0;
  right_encoder = 0;
  prev_left_encoder = 0;
  prev_right_encoder = 0;
  left_velocity = 0;
  right_velocity = 0;
}

void set_motor_output(float left, float right) {
  left = min(max(left, -1), 1);
  right = min(max(right, -1), 1);
  left_motor_output = (int) abs(round(left * 255));
  right_motor_output = (int) abs(round(right * 255));

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

long get_left_encoder_raw() {
  return left_encoder;
}

long get_right_encoder_raw() {
  return right_encoder;
}

float get_left_encoder() {
  return (float) left_encoder * DISTANCE_PER_TICK;
}

float get_right_encoder() {
  return (float) right_encoder * DISTANCE_PER_TICK;
}

float get_left_velocity() {
  return left_velocity;
}

float get_right_velocity() {
  return right_velocity;
}

float get_front_left_distance() {
  return 3500 - analogRead(PIN_DISTANCE_SENSOR_FRONT_LEFT);
}

float get_front_right_distance() {
  return 3500 - analogRead(PIN_DISTANCE_SENSOR_FRONT_RIGHT);
}

float get_rear_left_distance() {
  return 3500 - analogRead(PIN_DISTANCE_SENSOR_REAR_LEFT);
}

float get_rear_right_distance() {
  return 3500 - analogRead(PIN_DISTANCE_SENSOR_REAR_RIGHT);
}

void recompute_velocity(float dt) {
  float left_diff = ((float) (left_encoder - prev_left_encoder)) * DISTANCE_PER_TICK / dt;
  float right_diff = ((float) (right_encoder - prev_right_encoder)) * DISTANCE_PER_TICK / dt;
  prev_left_encoder = left_encoder;
  prev_right_encoder = right_encoder;
  left_velocity = VELOCITY_WINDOW_WEIGHT * left_velocity + (1.0 - VELOCITY_WINDOW_WEIGHT) * left_diff;
  right_velocity = VELOCITY_WINDOW_WEIGHT * right_velocity + (1.0 - VELOCITY_WINDOW_WEIGHT) * right_diff;
}
