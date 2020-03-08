void unicycle_to_differential_drive(float* left, float* right, float vel, float angular_vel, float radius, float wheel_base_length) {
  *left = (2 * vel - angular_vel * wheel_base_length) / (2 * radius);
  *right = (2 * vel + angular_vel * wheel_base_length) / (2 * radius);
}
