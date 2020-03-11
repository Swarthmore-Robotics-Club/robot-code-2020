/* Fake realities */
#define ORIENT_TIMES_NEEDED_ACCEPTABLE 10
#define MAX_ANGULAR_VELOCITY 32
#define ACCEPTABLE_DIFFERENCE 20

/* PID loop constants */
#define ORIENT_KP 0.0015
#define ORIENT_KI 0
#define ORIENT_KD 0.000375

PIDLoop* orientPID = new PIDLoop(ORIENT_KP, ORIENT_KI, ORIENT_KD);
int orientNumTimesAcceptable = 0;
const float thirtyOverPi = 30 / PI;

void orientYoSelfBeforeYouOccidentYoSelf(unsigned long dt) {
  float average = getDiff();
  float angularVelocity = orientPID->updateError(average, dt);
  float forwardVelocity = 0;
  
  float left, right;
  if (abs(average) < ACCEPTABLE_DIFFERENCE) {
    orientNumTimesAcceptable++;
    if (orientNumTimesAcceptable >= ORIENT_TIMES_NEEDED_ACCEPTABLE) {
      occident = DETERMINE;
      orientNumTimesAcceptable = 0;
      orientPID->wipe();
    }
    left = 0;
    right = 0;
  } else {
    unicycle_to_differential_drive(&left, &right, forwardVelocity, angularVelocity, WHEEL_RADIUS, WHEEL_BASE_LENGTH);
    setMotorDirection(left >= 0, true);
    setMotorDirection(right >= 0, false);
  }
  float leftVoltage = map(abs(left) * thirtyOverPi, 0, MAX_ANGULAR_VELOCITY, 0, MAX_VOLTAGE);
  float rightVoltage = map(abs(right) * thirtyOverPi, 0, MAX_ANGULAR_VELOCITY, 0, MAX_VOLTAGE);
  setMotors(leftVoltage, rightVoltage);
  prettyPrint("\taverage: " + (String)(average) + ", angular_vel: " + (String)(angularVelocity) + ", left: " + (String)(left) + ", right: " + (String)(right) + ", dt: " + (String)(dt) + "\n");  
}

/* Use the sensors that are closer to a wall, more accurate readings */
float getDiff() {
  if (sumBR + sumFR > sumFL + sumBL) {
    return (sumFR - sumBR) / NUM_TIMES_TO_READ_SENSORS;
  }
  return (sumBL - sumFL) / NUM_TIMES_TO_READ_SENSORS;
}
