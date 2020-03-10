#include "RobotInterface.h"

RobotInterface::RobotInterface() {}

RobotInterface::~RobotInterface() {}

void RobotInterface::doUpdate(float t, float dt) {
  if (t - velocityRecomputeTimer > velocityRecomputeEvery) {
    float leftDiff = (getLeftEncoder() - prevLeftEncoder) * getDistancePerTick() / dt;
    float rightDiff = (getRightEncoder() - prevRightEncoder) * getDistancePerTick() / dt;
    prevLeftEncoder = getLeftEncoder();
    prevRightEncoder = getRightEncoder();
    leftVelocity = velocityEmaCoefficient * leftVelocity + (1. - velocityEmaCoefficient) * leftDiff;
    rightVelocity = velocityEmaCoefficient * rightVelocity + (1. - velocityEmaCoefficient) * rightDiff;
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
