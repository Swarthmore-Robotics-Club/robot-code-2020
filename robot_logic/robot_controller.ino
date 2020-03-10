#include "robot_include.h"

float left_target_velocity;
float right_target_velocity;

float prev_left_error;
float prev_right_error;
float integral_left_error;
float integral_right_error;

float controller_kp_left = 0.031;
float controller_ki_left = 0.005;
float controller_kd_left = 0.0000;
float controller_kcalib_left = 0.0112;
float controller_kp_right = 0.03;
float controller_ki_right = 0.005;
float controller_kd_right = 0.0000;
float controller_kcalib_right = 0.0112;

float motion_profile_v_max;
float motion_profile_v_ramp_rate;
float motion_profile_start_time;
float motion_profile_rotate_angle;
float motion_profile_forward_distance;
float motion_profile_feedback_start_left;
float motion_profile_feedback_start_right;
float motion_profile_feedback_t0;

void setup_robot_controller() {
  prev_left_error = 0;
  prev_right_error = 0;
  integral_left_error = 0;
  integral_right_error = 0;
}

void set_velocity(float left, float right) {
  left_target_velocity = left;
  right_target_velocity = right;
}

void set_polar_velocity(float dr, float dtheta) {
  set_velocity(dr - (PI*ROBOT_RADIUS*dtheta), 
               dr + (PI*ROBOT_RADIUS*dtheta));
}

float get_left_target_velocity() {
  return left_target_velocity;
}

float get_right_target_velocity() {
  return right_target_velocity;
}

float run_velocity_pid_loop(float dt) {
  float left_error = left_target_velocity - get_left_velocity();
  float right_error = right_target_velocity - get_right_velocity();
  float diff_left_error = (left_error - prev_left_error) / dt;
  float diff_right_error = (right_error - prev_right_error) / dt;

  prev_left_error = left_error;
  prev_right_error = right_error;

  integral_left_error += left_error * dt;
  integral_right_error += right_error * dt;
  integral_left_error = min(max(integral_left_error, -1.0/controller_ki_left), 1.0/controller_ki_left);
  integral_right_error = min(max(integral_right_error, -1.0/controller_ki_right), 1.0/controller_ki_right);

  float left_motor_output = controller_kp_left * left_error 
                          + controller_ki_left * integral_left_error 
                          + controller_kd_left * diff_left_error 
                          + controller_kcalib_left * left_target_velocity;
  float right_motor_output = controller_kp_right * right_error 
                          + controller_ki_right * integral_right_error 
                          + controller_kd_right * diff_right_error 
                          + controller_kcalib_right * right_target_velocity;

  // sanity checks
  if (left_target_velocity == 0) {
    left_motor_output = 0;
  }

  if (right_target_velocity == 0) {
    right_motor_output = 0;
  }

  if (get_left_velocity() == 0 && abs(left_motor_output) > 0.15) {
    left_motor_output = (left_motor_output / abs(left_motor_output)) * 0.15;
  }

  if (get_right_velocity() == 0 && abs(right_motor_output) > 0.15) {
    right_motor_output = (right_motor_output / abs(right_motor_output)) * 0.15;
  }

  set_motor_output(left_motor_output, right_motor_output);
}

float ramp_velocity(float v_max, float v_ramp_rate, float t) {
  return max(0., min(v_max, v_ramp_rate * t));
}

void configure_motion_profiling_forward(
            float v_max, 
            float v_ramp_rate,
            float forward_distance) {
  motion_profile_v_max = v_max;
  motion_profile_v_ramp_rate = v_ramp_rate;
  motion_profile_start_time = micros() * 1e-6;
  motion_profile_forward_distance = forward_distance;
  motion_profile_feedback_start_left = get_left_encoder();
  motion_profile_feedback_start_right = get_right_encoder();
  motion_profile_feedback_t0 = 0;
}

void configure_motion_profiling_rotate(
            float v_max, 
            float v_ramp_rate,
            float rotate_angle) {
  motion_profile_v_max = v_max;
  motion_profile_v_ramp_rate = v_ramp_rate;
  motion_profile_start_time = micros() * 1e-6;
  motion_profile_rotate_angle = rotate_angle;
  motion_profile_feedback_start_left = get_left_encoder();
  motion_profile_feedback_start_right = get_right_encoder();
  motion_profile_feedback_t0 = 0;
}

void run_motion_profiling(float t) {
  robot_state_t state = get_robot_state();
  robot_velocity velocity_out;
  switch (state) {
    case RS_FORWARD: {
      float multiplier = motion_profile_forward_distance >= 0 ? 1 : -1;
      if (0.5 * (abs(get_left_encoder() - motion_profile_feedback_start_left)
              + abs(get_right_encoder() - motion_profile_feedback_start_right))
          >= abs(motion_profile_forward_distance / 2.) && motion_profile_feedback_t0 == 0) {
          motion_profile_feedback_t0 = t - motion_profile_start_time;
      }

      if (motion_profile_feedback_t0 == 0) {
        float v = multiplier * ramp_velocity(motion_profile_v_max, motion_profile_v_ramp_rate, t - motion_profile_start_time);
        set_velocity(v, v);
      } else {
        float v = multiplier * ramp_velocity(motion_profile_v_max, motion_profile_v_ramp_rate, 
               (2.*motion_profile_feedback_t0)
            - (t - motion_profile_start_time));
        set_velocity(v, v);
      }

      if (t - motion_profile_start_time > motion_profile_feedback_t0 * 2.0 && motion_profile_feedback_t0 > 0) {
        set_velocity(0, 0);
        set_robot_done(true);
      }
    }
    break;
    case RS_ROTATE: {
      float rotate_distance = abs(ROBOT_RADIUS * motion_profile_rotate_angle);
      float multiplier = motion_profile_rotate_angle >= 0 ? 1 : -1;
      if (0.5 * (abs(get_left_encoder() - motion_profile_feedback_start_left)
              + abs(get_right_encoder() - motion_profile_feedback_start_right))
          >= abs(rotate_distance / 2.) && motion_profile_feedback_t0 == 0) {
          motion_profile_feedback_t0 = t - motion_profile_start_time;
      }

      if (motion_profile_feedback_t0 == 0) {
        float v = multiplier * ramp_velocity(motion_profile_v_max, motion_profile_v_ramp_rate, t - motion_profile_start_time);
        set_velocity(v, -v);
      } else {
        float v = multiplier * ramp_velocity(motion_profile_v_max, motion_profile_v_ramp_rate, 
               (2.*motion_profile_feedback_t0)
            - (t - motion_profile_start_time));
        set_velocity(v, -v);
      }

      if (t - motion_profile_start_time > motion_profile_feedback_t0 * 2.0 && motion_profile_feedback_t0 > 0) {
        set_velocity(0, 0);
        set_robot_done(true);
      }
    }
    break;
    default:
      break;
  }
}
