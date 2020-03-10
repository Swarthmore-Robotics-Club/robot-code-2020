/* Fake realities for going forward */
#define RAMP_RATE 1 // voltage increase per loop

typedef struct {
  long desiredTicks;
  long halfwayPoint;
  long ticksAtFullSpeed;
  int currentVoltage;
} Forward;
Forward buzzLightyearLeftThruster;
Forward buzzLightyearRightThruster;
int timesAtZero = 0;
void toInfinityAndBeyond() {
  long left = leftEncoder;
  long right = rightEncoder;
  prettyPrint("Left encoder: " + (String)left + ", right encoder: " + (String)right + "\n");
  int newLeftVoltage, newRightVoltage;
  if (left < buzzLightyearLeftThruster.halfwayPoint) { /* Control for uneven motors...later by splitting into if statements for left and right, seeing if uneven # of ticks */
    newLeftVoltage = CAP_VOLTAGE(buzzLightyearLeftThruster.currentVoltage + RAMP_RATE);
    newRightVoltage = CAP_VOLTAGE(buzzLightyearRightThruster.currentVoltage + RAMP_RATE);
    if (newLeftVoltage == DESIRED_MAX_VOLTAGE) {
      buzzLightyearLeftThruster.ticksAtFullSpeed += 1;
      buzzLightyearRightThruster.ticksAtFullSpeed += 1;
    }
  } else {
    if (buzzLightyearLeftThruster.ticksAtFullSpeed != 0) { /* Keep going at full speed - don't need to decelerate yet */
      buzzLightyearLeftThruster.ticksAtFullSpeed -= 1;
      newLeftVoltage = buzzLightyearLeftThruster.currentVoltage;
      buzzLightyearRightThruster.ticksAtFullSpeed -= 1;
      newRightVoltage = buzzLightyearRightThruster.currentVoltage;
    } else {
      newLeftVoltage = CAP_VOLTAGE(buzzLightyearLeftThruster.currentVoltage - RAMP_RATE);
      newRightVoltage = CAP_VOLTAGE(buzzLightyearRightThruster.currentVoltage - RAMP_RATE);
    }
  }
  prettyPrint("ticks at full speed:" + (String)buzzLightyearLeftThruster.ticksAtFullSpeed + "\n");
  setMotors(newLeftVoltage, newRightVoltage);
  buzzLightyearLeftThruster.currentVoltage = newLeftVoltage;
  buzzLightyearRightThruster.currentVoltage = newRightVoltage;
  if (newLeftVoltage == 0) {
    timesAtZero += 1;
    if (timesAtZero > 10) occident = ORIENT;
  }
  return;
}
