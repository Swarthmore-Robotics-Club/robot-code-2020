/*
 * forward_vel = meters / second
 * angular_vel = radians / second
 * wheel_* = meters
 * 
 * out = radians / second
 */
void unicycle_to_differential_drive(float* left, float* right, float forward_vel, float angular_vel, float wheel_radius, float wheel_base_length) {
  *left = ((2 * forward_vel) - (angular_vel * wheel_base_length)) / (2 * wheel_radius);
  *right = ((2 * forward_vel) + (angular_vel * wheel_base_length)) / (2 * wheel_radius);
}
