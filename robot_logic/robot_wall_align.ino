#include "robot_include.h"

float prev_wall_align_error;
float integral_wall_align_error;

float wall_align_kp = 0.001;
float wall_align_ki = 0.00002;
float wall_align_kd = 0.00;

void setup_robot_wall_align() {
  reset_robot_wall_align();
}

void reset_robot_wall_align() {
  prev_wall_align_error = 0;
  integral_wall_align_error = 0;
}

void run_robot_wall_align(float dt) {
  if (get_robot_state() == RS_WALL_ALIGN) {
    
    float error = get_front_left_distance() - get_rear_left_distance();
    float diff_error = (error - prev_wall_align_error) / dt;
    prev_wall_align_error = error;
  
    integral_wall_align_error += error * dt;
    integral_wall_align_error = min(max(integral_wall_align_error, -1.0/wall_align_ki), 1.0/wall_align_ki);
  
    float dtheta = wall_align_kp * error 
                 + wall_align_ki * integral_wall_align_error 
                 + wall_align_kd * diff_error;

    if (fabs(error) < 100) {
      dtheta = 0;
    }

    set_polar_velocity(0, dtheta);
  } else {
    reset_robot_wall_align();
  }
}
