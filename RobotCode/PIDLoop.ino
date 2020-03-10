#include "PIDLoop.h"

PIDLoop::PIDLoop(float kP1, float kI1, float kD1, float kF1) {
  kP = kP1;
  kI = kI1;
  kD = kD1;
  kF = kF1;
}

float PIDLoop::updateError(float target, float actual, unsigned long dt) {
  float error = target - actual;
  float oldError = pError;
  pError = error;
  if (error == 0) {
    iError = 0;
  } else {
    iError += (error * dt);
  }
  dError = (error - oldError) / dt;
  return kF * target + kP * pError + kI * iError + kD * dError;
}

void PIDLoop::wipe() {
  pError = 0;
  iError = 0;
  dError = 0;
}
