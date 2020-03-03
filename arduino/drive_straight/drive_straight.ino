#include <stdio.h>
#include <math.h>
#include <arduino.h>

// MEASUREMENTS /////////////////////////////////////

// note: all units in cm

#define ROBOT_RADIUS 4.8
#define ROBOT_WIDTH (ROBOT_RADIUS*2.0)
#define ROBOT_CIRCUMFERENCE (ROBOT_WIDTH * PI)
#define WHEEL_RADIUS 2.5
#define WHEEL_CIRCUMFERENCE (2. * WHEEL_RADIUS * PI)
#define TICKS_PER_REVOLUTION 225.
#define DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION)

// ENCODER / VELOCITY ///////////////////////////////

//#define DISABLE_ROBOT

#define MOTOR_A1 6
#define MOTOR_A2 7
#define MOTOR_B1 8
#define MOTOR_B2 9
#define MOTOR_APWM 11
#define MOTOR_BPWM 10

#define ENCODER_B1 3
#define ENCODER_B2 5
#define ENCODER_A1 2
#define ENCODER_A2 4

#define DEPTHSENSOR_A A3
#define DEPTHSENSOR_B A0
#define DEPTHSENSOR_C A7
#define DEPTHSENSOR_D A5

#define VELOCITY_WINDOW_WEIGHT (0.5)

long prev_encoder_a_x = 0;
long prev_encoder_b_x = 0;
long encoder_a_x = 0;
long encoder_b_x = 0;
float encoder_a = 0;
float encoder_b = 0;
float velocity_a = 0;
float velocity_b = 0;
float target_vel_a = 0.0;
float target_vel_b = 0.0;

float v_max = 20;
float v_ramp_rate = 50;

// SENSORS //////////////////////////////////////////////
int distance_ll = 0;
int distance_rr = 0;
int distance_lc = 0;
int distance_rc = 0;

// PID //////////////////////////////////////////////
float motor_a = 0;
float motor_b = 0;
float prev_error_a = 0;
float prev_error_b = 0;
float intg_error_a = 0;
float intg_error_b = 0;

float KP_A = 0.025;
float KI_A = 0.0018;
float KD_A = 0.000;
float KCALIB_A = 0.009;
float KP_B = 0.030;
float KI_B = 0.0021;
float KD_B = 0.000;
float KCALIB_B = 0.010;

// TIMING //////////////////////////////////////////
unsigned long prev_time = 0;

// STATE MACHINE ///////////////////////////////////

#define SM_STATE_DISABLED 0
#define SM_STATE_VELOCITY 1
#define SM_STATE_POSITION_VELOCITY 2
#define SM_STATE_MOTION_PROFILE 3

#define SM_MOTION_PROFILE_NONE 0
#define SM_MOTION_PROFILE_FORWARD 1
#define SM_MOTION_PROFILE_ROTATE 2
#define SM_MOTION_PROFILE_FEEDBACK_FORWARD 3
#define SM_MOTION_PROFILE_FEEDBACK_ROTATE 4

unsigned int sm_state = SM_STATE_MOTION_PROFILE;
unsigned int sm_motion_profile = SM_MOTION_PROFILE_NONE;

float sm_motion_profile_start_time = 0;
float sm_motion_profile_forward_distance = 0;
float sm_motion_profile_rotate_angle = 0;
float sm_motion_profile_feedback_forward_start_left = 0;
float sm_motion_profile_feedback_forward_start_right = 0;
float sm_motion_profile_feedback_forward_t0 = 0;

// POSITION CONTROL ////////////////////////////////

float pc_left_target = 0;
float pc_right_target = 0;
float pc_velocity_max = 0;

void encoder_a_tick();
void encoder_b_tick();
void run_velocity_pid(float);

// MOTION PROFILING ////////////////////////////////

typedef struct {
  float left_velocity;
  float right_velocity;
} robot_velocity;

int motion_profile_forward(
        float distance, 
        float v_max, 
        float v_ramp_rate, 
        float t, 
        robot_velocity* velocity_out) {
  float ramp_time = v_max / v_ramp_rate;
  float v_max_time = (distance / v_max) - ramp_time;
  if (v_max_time >= 0) {
    if (t < ramp_time) {
      velocity_out->left_velocity = v_ramp_rate * t;
      velocity_out->right_velocity = v_ramp_rate * t;
      return 0;
    } else if (t < ramp_time + v_max_time) {
      velocity_out->left_velocity = v_max;
      velocity_out->right_velocity = v_max;
      return 0;
    } else if (t < ramp_time + v_max_time + ramp_time) {
      velocity_out->left_velocity = v_max - (v_ramp_rate * (t - (ramp_time + v_max_time)));
      velocity_out->right_velocity = v_max - (v_ramp_rate * (t - (ramp_time + v_max_time)));
      return 0;
    } else {
      velocity_out->left_velocity = 0;
      velocity_out->right_velocity = 0;
      return 1;
    }
  } else {
    float ramp_time = sqrt(distance / v_ramp_rate);
    float peak_velocity = ramp_time * v_ramp_rate;
    if (t < ramp_time) {
      velocity_out->left_velocity = t * v_ramp_rate;
      velocity_out->right_velocity = t * v_ramp_rate;
      return 0;
    } else if (t < ramp_time + ramp_time) {
      velocity_out->left_velocity = peak_velocity - (t - ramp_time) * v_ramp_rate;
      velocity_out->right_velocity = peak_velocity - (t - ramp_time) * v_ramp_rate;
      return 0;
    } else {
      velocity_out->left_velocity = 0;
      velocity_out->right_velocity = 0;
      return 1;
    }
  }
}

int motion_profile_rotate(
        float angle, 
        float v_max, 
        float v_ramp_rate, 
        float t, 
        robot_velocity* velocity_out) {
  float distance = ROBOT_RADIUS * angle;
  motion_profile_forward(distance, v_max, v_ramp_rate, t, velocity_out);
  if (angle >= 0) {
    velocity_out->left_velocity *= -1;
  } else {
    velocity_out->right_velocity *= -1;
  }
}

float ramp_velocity(float v_max, float v_ramp_rate, float t) {
  return max(0., min(v_max, v_ramp_rate * t));
}

float get_front_right_distance(int voltage) {
  float voltagef = (float) voltage;
  // calibrated with best fit in excel
  return 0.120 * (488 - voltagef);
}

float get_front_left_distance(int voltage) {
  float voltagef = (float) voltage;
  // assume left calibration = right calibration approximately
  return 0.1283219341 * (488 - voltagef);
}

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(MOTOR_APWM, OUTPUT);
  pinMode(MOTOR_BPWM, OUTPUT);
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(DEPTHSENSOR_A, INPUT);
  pinMode(DEPTHSENSOR_B, INPUT);
  pinMode(DEPTHSENSOR_C, INPUT);
  pinMode(DEPTHSENSOR_D, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder_a_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoder_b_tick, RISING);

  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_APWM, 0);
  analogWrite(MOTOR_BPWM, 0);

  prev_time = micros();
  Serial.begin(9600);
}

void encoder_a_tick() {
  encoder_a_x += -(1 - 2*digitalRead(ENCODER_A2));
  encoder_a = encoder_a_x * DISTANCE_PER_TICK;
}

void encoder_b_tick() {
  encoder_b_x += (1 - 2*digitalRead(ENCODER_B2));
  encoder_b = encoder_b_x * DISTANCE_PER_TICK;
}

void run_velocity_pid(float dt) {
  float error_a = target_vel_a - velocity_a;
  float error_b = target_vel_b - velocity_b;
  float error_diff_a = (error_a - prev_error_a) / dt;
  float error_diff_b = (error_b - prev_error_b) / dt;
  
  prev_error_a = error_a;
  prev_error_b = error_b;
  intg_error_a += error_a * dt;
  intg_error_b += error_b * dt;
  intg_error_a = min(max(intg_error_a, -1.0/KI_A), 1.0/KI_A);
  intg_error_b = min(max(intg_error_b, -1.0/KI_B), 1.0/KI_B);

  motor_a = KP_A * error_a + KI_A * intg_error_a + KD_A * error_diff_a + KCALIB_A * target_vel_a;
  motor_b = KP_B * error_b + KI_B * intg_error_b + KD_B * error_diff_b + KCALIB_B * target_vel_b;

  motor_a = min(max(motor_a, -1.0), 1.0);
  motor_b = min(max(motor_b, -1.0), 1.0);

  // sanity checks
  if (target_vel_a == 0) {
    motor_a = 0;
  }

  if (target_vel_b == 0) {
    motor_b = 0;
  }

  if (velocity_a == 0 && abs(motor_a) > 0.25) {
    motor_a = (motor_a / abs(motor_a)) * 0.25;
  }

  if (velocity_b == 0 && abs(motor_b) > 0.25) {
    motor_b = (motor_b / abs(motor_b)) * 0.25;
  }
}

void update_velocity(float dt) {
  float a_diff = (((float) (encoder_a_x - prev_encoder_a_x)) * DISTANCE_PER_TICK) / dt;
  float b_diff = (((float) (encoder_b_x - prev_encoder_b_x)) * DISTANCE_PER_TICK) / dt;
  velocity_a = VELOCITY_WINDOW_WEIGHT * velocity_a + (1.0 - VELOCITY_WINDOW_WEIGHT) * a_diff;
  velocity_b = VELOCITY_WINDOW_WEIGHT * velocity_b + (1.0 - VELOCITY_WINDOW_WEIGHT) * b_diff;

  prev_encoder_a_x = encoder_a_x;
  prev_encoder_b_x = encoder_b_x;
}

void publish_velocity() {
  int motor_a_out = (int) round(255.0 * motor_a);
#ifdef DISABLE_ROBOT
  motor_a_out = 0;
#endif
  if (sm_state == SM_STATE_DISABLED) {
    motor_a_out = 0;
  }
  if (motor_a_out > 0) {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    analogWrite(MOTOR_APWM, motor_a_out);
  } else if (motor_a_out < 0) {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    analogWrite(MOTOR_APWM, -motor_a_out);
  } else {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    analogWrite(MOTOR_APWM, 0);
  }

  int motor_b_out = (int) round(255.0 * motor_b);
#ifdef DISABLE_ROBOT
  motor_b_out = 0;
#endif
  if (sm_state == SM_STATE_DISABLED) {
    motor_b_out = 0;
  }
  if (motor_b_out > 0) {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_BPWM, motor_b_out);
  } else if (motor_b_out < 0) {
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(MOTOR_BPWM, -motor_b_out);
  } else {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(MOTOR_BPWM, 0);
  }
}

void update_distance() {
  distance_ll = analogRead(DEPTHSENSOR_A);
  distance_rr = analogRead(DEPTHSENSOR_B);
  distance_lc = analogRead(DEPTHSENSOR_D);
  distance_rc = analogRead(DEPTHSENSOR_C);
}

void run_motion_profiling(float t) {
  if (sm_state == SM_STATE_MOTION_PROFILE) {
    robot_velocity velocity_out;
    switch (sm_motion_profile) {
      case SM_MOTION_PROFILE_NONE:
        target_vel_a = 0;
        target_vel_b = 0;
        break;
      case SM_MOTION_PROFILE_FORWARD:
        motion_profile_forward(sm_motion_profile_forward_distance, 
                    v_max,
                    v_ramp_rate,
                    t - sm_motion_profile_start_time,
                    &velocity_out);
        target_vel_b = velocity_out.left_velocity;
        target_vel_a = velocity_out.right_velocity;
        break;
      case SM_MOTION_PROFILE_ROTATE:
        motion_profile_rotate(sm_motion_profile_rotate_angle, 
                    v_max,
                    v_ramp_rate,
                    t - sm_motion_profile_start_time,
                    &velocity_out);
        target_vel_b = velocity_out.left_velocity;
        target_vel_a = velocity_out.right_velocity;
        break;
      case SM_MOTION_PROFILE_FEEDBACK_FORWARD:
        float forward_multiplier = sm_motion_profile_forward_distance >= 0 ? 1 : -1;
        if (
                0.5 * (abs(encoder_a - sm_motion_profile_feedback_forward_start_left)
                     + abs(encoder_b - sm_motion_profile_feedback_forward_start_right))
              >= abs(sm_motion_profile_forward_distance / 2.) && sm_motion_profile_feedback_forward_t0 == 0) {
          sm_motion_profile_feedback_forward_t0 = t - sm_motion_profile_start_time;
        }

        if (sm_motion_profile_feedback_forward_t0 == 0) {
          target_vel_b = forward_multiplier * ramp_velocity(v_max, v_ramp_rate, t - sm_motion_profile_start_time);
          target_vel_a = forward_multiplier * ramp_velocity(v_max, v_ramp_rate, t - sm_motion_profile_start_time);
        } else {
          target_vel_b = forward_multiplier * ramp_velocity(v_max, v_ramp_rate, 
                 (2.*sm_motion_profile_feedback_forward_t0)
              - (t - sm_motion_profile_start_time));
          target_vel_a = forward_multiplier * ramp_velocity(v_max, v_ramp_rate, 
                 (2.*sm_motion_profile_feedback_forward_t0)
              - (t - sm_motion_profile_start_time));
        }
        break;
      case SM_MOTION_PROFILE_FEEDBACK_ROTATE:
        float rotate_distance = abs(ROBOT_RADIUS * sm_motion_profile_rotate_angle);
        float rotate_multiplier = sm_motion_profile_rotate_angle >= 0 ? 1 : -1;
        if (
                0.5 * (abs(encoder_a - sm_motion_profile_feedback_forward_start_left)
                     + abs(encoder_b - sm_motion_profile_feedback_forward_start_right))
              >= abs(rotate_distance / 2.) && sm_motion_profile_feedback_forward_t0 == 0) {
          sm_motion_profile_feedback_forward_t0 = t - sm_motion_profile_start_time;
        }

        if (sm_motion_profile_feedback_forward_t0 == 0) {
          target_vel_b = (-rotate_multiplier) * ramp_velocity(v_max, v_ramp_rate, t - sm_motion_profile_start_time);
          target_vel_a = rotate_multiplier * ramp_velocity(v_max, v_ramp_rate, t - sm_motion_profile_start_time);
        } else {
          target_vel_b = (-rotate_multiplier) * ramp_velocity(v_max, v_ramp_rate, 
                 (2.*sm_motion_profile_feedback_forward_t0)
              - (t - sm_motion_profile_start_time));
          target_vel_a = rotate_multiplier * ramp_velocity(v_max, v_ramp_rate, 
                 (2.*sm_motion_profile_feedback_forward_t0)
              - (t - sm_motion_profile_start_time));
        }
        break;
      default:
        break;
    }
  }
}

int logic_state = 0;
void run_logic(float t) {
  sm_state = SM_STATE_MOTION_PROFILE;

//      sm_motion_profile = SM_MOTION_PROFILE_FEEDBACK_ROTATE;
//    sm_motion_profile = SM_MOTION_PROFILE_FORWARD;
//    sm_motion_profile = SM_MOTION_PROFILE_ROTATE;
//    sm_motion_profile = SM_MOTION_PROFILE_NONE;

  if (fmod(t, 20) > 10) {
    if (logic_state != 1) {
      sm_motion_profile = SM_MOTION_PROFILE_FEEDBACK_FORWARD;
      sm_motion_profile_start_time = t;
      sm_motion_profile_forward_distance = 90.0;
      sm_motion_profile_feedback_forward_start_right = encoder_b;
      sm_motion_profile_feedback_forward_start_left = encoder_a;
      sm_motion_profile_feedback_forward_t0 = 0;
      logic_state = 1;
    }
  } else if (fmod(t, 20) > 0) {
    if (logic_state != 2) {
      sm_motion_profile = SM_MOTION_PROFILE_FEEDBACK_FORWARD;
      sm_motion_profile_start_time = t;
      sm_motion_profile_forward_distance = -90.0;
      sm_motion_profile_feedback_forward_start_right = encoder_b;
      sm_motion_profile_feedback_forward_start_left = encoder_a;
      sm_motion_profile_feedback_forward_t0 = 0;
      logic_state = 2;
    }
  }
  
//    sm_motion_profile = SM_MOTION_PROFILE_NONE;
//    sm_motion_profile_start_time = t;
//    sm_motion_profile_rotate_angle = (-PI / 2.) ;//* 1.02; //* 1.037;
//    sm_motion_profile_forward_distance = 30.0;
//    sm_motion_profile_feedback_forward_start_right = encoder_b;
//    sm_motion_profile_feedback_forward_start_left = encoder_a;
//    sm_motion_profile_feedback_forward_t0 = 0;
  
}

void loop() {
  // TIMING
  
  float dt = (float) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  float t = prev_time * 1e-6;

  update_velocity(dt);
  update_distance();
  run_logic(t);
  run_motion_profiling(t);
  run_velocity_pid(dt);
  publish_velocity();

  Serial.print(encoder_b);
  Serial.print("\t");
  Serial.print(encoder_a);
  Serial.println();
  
//  delay(10);
}
