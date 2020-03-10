#include "RobotInterface.h"

RobotInterface::RobotInterface() {}

RobotInterface::~RobotInterface() {}

void RobotInterface::doUpdate(double t, double dt) {
  if (t - velocityRecomputeTimer > velocityRecomputeEvery) {
    float leftDiff = (getLeftEncoder() - prevLeftEncoder) / (t - velocityRecomputeTimer);
    float rightDiff = (getRightEncoder() - prevRightEncoder) / (t - velocityRecomputeTimer);
    prevLeftEncoder = getLeftEncoder();
    prevRightEncoder = getRightEncoder();
    leftVelocity = velocityEmaCoefficient * leftVelocity + (1. - velocityEmaCoefficient) * leftDiff;
    rightVelocity = velocityEmaCoefficient * rightVelocity + (1. - velocityEmaCoefficient) * rightDiff;
    velocityRecomputeTimer = t;
  }
}

float RobotInterface::getLeftEncoder() {
  return getLeftEncoderRaw() * getDistancePerTick();
}

float RobotInterface::getRightEncoder() {
  return getRightEncoderRaw() * getDistancePerTick();
}

float RobotInterface::getLeftVelocity() {
  return leftVelocity;
}

float RobotInterface::getRightVelocity() {
  return rightVelocity;
}

float RobotInterface::getDistancePerTick() {
  return 2. * PI * getWheelRadius() / getTicksPerRevolution();
}
