float left_target_velocity;
float right_target_velocity;

float prev_left_error;
float prev_right_error;
float integral_left_error;
float integral_right_error;

//float kp_left = 0.025;
//float ki_left = 0.0018;
//float kd_left = 0.000;
//float kcalib_left = 0.009;
//float kp_right = 0.030;
//float ki_right = 0.0021;
//float kd_right = 0.000;
//float kcalib_right = 0.010;

float kp_left = 0.015;
float ki_left = 0.05;
float kd_left = 0.0000;
float kcalib_left = 0.015;
float kp_right = 0.015;
float ki_right = 0.05;
float kd_right = 0.0000;
float kcalib_right = 0.015;

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
  integral_left_error = min(max(integral_left_error, -1.0/ki_left), 1.0/ki_left);
  integral_right_error = min(max(integral_right_error, -1.0/ki_right), 1.0/ki_right);

  float left_motor_output = kp_left * left_error 
                          + ki_left * integral_left_error 
                          + kd_left * diff_left_error 
                          + kcalib_left * left_target_velocity;
  float right_motor_output = kp_right * right_error 
                          + ki_right * integral_right_error 
                          + kd_right * diff_right_error 
                          + kcalib_right * right_target_velocity;

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
