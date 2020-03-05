#include "robot_include.h"

robot_state_t robot_state;
bool robot_state_done;
float state_machine_timer;

void setup_robot_state_machine() {
  robot_state = RS_IDLE;
  robot_state_done = false;
  state_machine_timer = micros() * 1e-6;
}

bool is_robot_done() {
  return robot_state_done;
}

bool set_robot_done(bool done) {
  robot_state_done = done;
}

void set_robot_state(robot_state_t state) {
  robot_state = state;
  set_robot_done(false);
}

robot_state_t get_robot_state() {
  return robot_state;
}

void run_state_machine(float t) {
  switch (robot_state) {
    case RS_IDLE:
      set_robot_state(RS_PRE_FORWARD);
      break;
    case RS_FORWARD:
      if (is_robot_done()) {
        state_machine_timer = t;
        set_robot_state(RS_PRE_ROTATE);
      }
      break;
    case RS_ROTATE:
      if (is_robot_done()) {
        state_machine_timer = t;
        set_robot_state(RS_PRE_FORWARD);
      }
      break;
    case RS_PRE_FORWARD:
      if (t - state_machine_timer > 0.5) {
        configure_motion_profiling_forward(V_MAX, V_RAMP_RATE, 50);
        set_robot_state(RS_FORWARD);
      }
      break;
    case RS_PRE_ROTATE:
      if (t - state_machine_timer > 0.5) {
        configure_motion_profiling_rotate(V_MAX, V_RAMP_RATE, PI / 2.);
        set_robot_state(RS_ROTATE);
      }
      break;
    default:
      break;
  }
}
