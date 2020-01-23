#include <stdio.h>
#include <math.h>
#include <arduino.h>

// MEASUREMENTS /////////////////////////////////////

// note: all units in cm

#define ROBOT_WIDTH 9.6
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

#define DEPTHSENSOR_A A2
#define DEPTHSENSOR_B A0

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

// SENSORS //////////////////////////////////////////////
int distance_ll = 0;
int distance_rr = 0;


// PID //////////////////////////////////////////////
float motor_a = 0;
float motor_b = 0;
float prev_error_a = 0;
float prev_error_b = 0;
float intg_error_a = 0;
float intg_error_b = 0;
float KP_A = 0;
float KI_A = 0;
float KD_A = 0;
float KCALIB_A = 0;
float KP_B = 0;
float KI_B = 0;
float KD_B = 0;
float KCALIB_B = 0;

// TIMING //////////////////////////////////////////
unsigned long prev_time = 0;

// MESSAGING ///////////////////////////////////////
#define RPI_COMM_WATCHDOG_TIMEOUT (0.5)
#define RPI_COMM_UPDATE_TIMEOUT (0.1)
#define RPI_COMM_MESSAGE_WATCHDOG 0
#define RPI_COMM_MESSAGE_VELOCITY 1
#define RPI_COMM_MESSAGE_KP_A 2
#define RPI_COMM_MESSAGE_KI_A 3
#define RPI_COMM_MESSAGE_KD_A 4
#define RPI_COMM_MESSAGE_KCALIB_A 5
#define RPI_COMM_MESSAGE_KP_B 6
#define RPI_COMM_MESSAGE_KI_B 7
#define RPI_COMM_MESSAGE_KD_B 8
#define RPI_COMM_MESSAGE_KCALIB_B 9
#define RPI_COMM_UPDATE_NULL 0
#define RPI_COMM_UPDATE_ENCODER_RIGHT 1
#define RPI_COMM_UPDATE_ENCODER_LEFT 2
#define RPI_COMM_UPDATE_DISTANCE_RR 3
#define RPI_COMM_UPDATE_DISTANCE_LL 4
#define RPI_COMM_MESSAGE_ROBOT_INIT 114   // 'r' in 'robotics'


String rpi_comm_string = "";
bool rpi_comm_started = false;
float rpi_comm_watchdog_time = 0.0;
byte rpi_comm_message_type = 0;
float rpi_comm_update_time = 0.0;
bool rpi_comm_restart_sending = false;

// STATE MACHINE ///////////////////////////////////

#define SM_STATE_DISABLED 0
#define SM_STATE_VELOCITY 1
#define SM_STATE_POSITION_VELOCITY 2
unsigned int sm_state = SM_STATE_DISABLED;

// POSITION CONTROL ////////////////////////////////

float pc_left_target = 0;
float pc_right_target = 0;
float pc_velocity_max = 0;

void encoder_a_tick();
void encoder_b_tick();
void run_velocity_pid(float);

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

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder_a_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoder_b_tick, RISING);

  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_APWM, 0);
  analogWrite(MOTOR_BPWM, 0);

  prev_time = micros();

  rpi_comm_string.reserve(128);

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

void run_messaging(float dt) {
  // RPI MESSAGING
  while (Serial.available()) {
    char cur_byte = (char) Serial.read();
    rpi_comm_string += cur_byte;
    rpi_comm_watchdog_time = 0.0;
    
    if (rpi_comm_message_type == 0) {
      rpi_comm_message_type = cur_byte;
      rpi_comm_string = "";
    } else {
      switch (rpi_comm_message_type) {
      case RPI_COMM_MESSAGE_VELOCITY:
        if (rpi_comm_string.length() >= sizeof(float)*2 && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          target_vel_a = *((float*) c_rpi_comm_string);
          target_vel_b = *((float*) (c_rpi_comm_string+sizeof(float)));
          sm_state = SM_STATE_VELOCITY;
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KP_A:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KP_A = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KI_A:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KI_A = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KD_A:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KD_A = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KCALIB_A:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KCALIB_A = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KP_B:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KP_B = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KI_B:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KI_B = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KD_B:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KD_B = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_KCALIB_B:
        if (rpi_comm_string.length() >= sizeof(float) && rpi_comm_started) {
          char* c_rpi_comm_string = rpi_comm_string.c_str();
          KCALIB_B = *((float*) c_rpi_comm_string);
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      case RPI_COMM_MESSAGE_ROBOT_INIT:
        if (rpi_comm_string.length() >= 7) {
          if (rpi_comm_string.equals("obotics")) {
            rpi_comm_restart_sending = true;
            rpi_comm_started = true;
          }
          rpi_comm_string = "";
          rpi_comm_message_type = 0;
        }
        break;
      default:
        rpi_comm_message_type = 0;
        rpi_comm_string = "";
        break;
      }
    }
  }

  rpi_comm_update_time += dt;
  if (rpi_comm_update_time >= RPI_COMM_UPDATE_TIMEOUT) {
    rpi_comm_update_time -= RPI_COMM_UPDATE_TIMEOUT;

    if (rpi_comm_started) {
      if (rpi_comm_restart_sending) {
        rpi_comm_restart_sending = false;
        for (int i=0; i<32; ++i) {
          Serial.write(RPI_COMM_UPDATE_NULL);
        }
      }
  
      Serial.write(RPI_COMM_UPDATE_ENCODER_RIGHT);
      Serial.write((byte*) &encoder_a, sizeof(encoder_a));
      Serial.write(RPI_COMM_UPDATE_ENCODER_LEFT);
      Serial.write((byte*) &encoder_b, sizeof(encoder_b));
      Serial.write(RPI_COMM_UPDATE_DISTANCE_RR);
      Serial.write((byte*) &encoder_a, sizeof(encoder_a));
      Serial.write(RPI_COMM_UPDATE_DISTANCE_LL);
      Serial.write((byte*) &encoder_a, sizeof(encoder_a));
    }
  }
}

void run_watchdog(float dt) {
  rpi_comm_watchdog_time += dt;
  if (rpi_comm_watchdog_time > RPI_COMM_WATCHDOG_TIMEOUT) {
    target_vel_a = 0;
    target_vel_b = 0;
    sm_state = SM_STATE_DISABLED;
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
  
} 

void loop() {
  // TIMING

  float dt = (float) (micros() - prev_time) * 1e-6;
  prev_time = micros();

  run_messaging(dt);
  run_watchdog(dt);
  update_velocity(dt);
  update_distance();
  run_velocity_pid(dt);
  publish_velocity();

  delay(10);
}
