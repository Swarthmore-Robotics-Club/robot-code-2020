#include <arduino.h>
//#include <SoftwareSerial.h>

#define PIN_MOTOR_LEFT_PWM PB0
#define PIN_MOTOR_RIGHT_PWM PB1
#define PIN_MOTOR_LEFT_A PB5
#define PIN_MOTOR_LEFT_B PA8
#define PIN_MOTOR_RIGHT_A PB7
#define PIN_MOTOR_RIGHT_B PB6

#define PIN_ENCODER_LEFT_A PA6
#define PIN_ENCODER_LEFT_B PA7
#define PIN_ENCODER_RIGHT_A PA9
#define PIN_ENCODER_RIGHT_B PA10

#define PIN_DISTANCE_SENSOR_FRONT_LEFT PA0
#define PIN_DISTANCE_SENSOR_FRONT_RIGHT PA1
#define PIN_DISTANCE_SENSOR_REAR_LEFT PA11
#define PIN_DISTANCE_SENSOR_REAR_RIGHT PA5
#define PIN_DISTANCE_SENSOR_FRONT PA4

#define ROBOT_RADIUS 4.8
#define ROBOT_WIDTH (ROBOT_RADIUS*2.0)
#define ROBOT_CIRCUMFERENCE (ROBOT_WIDTH * PI)
#define WHEEL_RADIUS 2
#define WHEEL_CIRCUMFERENCE (2. * WHEEL_RADIUS * PI)
#define TICKS_PER_REVOLUTION 900
#define DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION)
#define VELOCITY_WINDOW_WEIGHT (0.5)

typedef struct {
  float left_velocity;
  float right_velocity;
} robot_velocity;

// state

unsigned long prev_time;

// state //

void setup() {
  setup_robot_interface();
  setup_robot_controller();
  setup_robot_state_machine();

  prev_time = micros();
  Serial2.begin(9600);
}

float sum_dt = 0;
void debug(float dt) {
  sum_dt += dt;
  if (sum_dt > 0.05) {
    Serial2.print("velocity ");
    Serial2.print(get_left_velocity());
    Serial2.print(" ");
    Serial2.print(get_right_velocity());
    Serial2.println();
    sum_dt = 0;
  }
}

void loop() {
  float dt = (float) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  float t = prev_time * 1e-6;

  recompute_velocity(dt);
  run_velocity_pid_loop(dt);
  
  debug(dt);

  delay(10);
}
