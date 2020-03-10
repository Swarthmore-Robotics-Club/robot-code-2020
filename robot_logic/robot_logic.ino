#include <arduino.h>
#include "robot_include.h"

// state

unsigned long prev_time;

// state //

void setup() {
  setup_robot_interface();
  setup_robot_controller();
  setup_robot_state_machine();
  setup_robot_wall_align();

  prev_time = micros();
  Serial2.begin(9600);
}

float sum_dt = 0;
void debug(float dt, float t) {
  sum_dt += dt;
  if (sum_dt > 0.1) {
//    Serial2.print("velocity ");
//    Serial2.print(get_left_velocity());
//    Serial2.print(" ");
//    Serial2.print(get_right_velocity());
    Serial2.print(" positioning ");
    Serial2.print(get_front_left_distance());
    Serial2.print(" ");
    Serial2.print(get_rear_left_distance());
    Serial2.print(" ");
    Serial2.print(get_front_right_distance());
    Serial2.print(" ");
    Serial2.print(get_rear_right_distance());
    Serial2.println();
    sum_dt = 0;
  }
}

void loop() {
  float dt = (float) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  float t = prev_time * 1e-6;

  run_state_machine(t);
  recompute_velocity(dt);
  run_motion_profiling(t);
  run_velocity_pid_loop(dt);
  run_robot_wall_align(dt);

  set_robot_state(RS_WALL_ALIGN);
  
  debug(dt, t);

  delay(10);
}
