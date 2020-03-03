typedef enum {
  RS_IDLE,
  RS_FORWARD,
  RS_BACKWARD,
  RS_RIGHT,
  RS_LEFT
} robot_state_t;

robot_state_t robot_state;
bool robot_state_done;

void setup_robot_state_machine() {
  
}

bool is_robot_done() {
  return robot_state_done;
}

void run_state_machine() {
  switch (robot_state) {
    case RS_IDLE:
      break;
    case RS_FORWARD:
      
  }
}
