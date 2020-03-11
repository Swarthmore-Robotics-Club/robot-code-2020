#include "VelocityController.h"

VelocityController::VelocityController(RobotInterface* iface) :
  leftPidLoop(0.08, 0.0001, 0., 0.0122),
  rightPidLoop(0.05, 0.0001, 0., 0.0112),
  RobotController(iface) {
}

VelocityController::~VelocityController() {}

void VelocityController::doUpdate(double t, double dt) {
  float left = leftPidLoop.updateError(leftTargetVelocity, interface->getLeftVelocity(), dt);
  float right = rightPidLoop.updateError(rightTargetVelocity, interface->getRightVelocity(), dt);

  // prevents oscillations around 0
  if (leftTargetVelocity == 0) {
    left = 0;
  }
  if (rightTargetVelocity == 0) {
    right = 0;
  }

  // edge case: if encoder breaks and velocity is always 0,
  // don't go above 0.15 motor output
  if (interface->getLeftVelocity() == 0 && abs(left) > 0.15) {
    left = (left / abs(left)) * 0.15;
  }
  if (interface->getRightVelocity() == 0 && abs(right) > 0.15) {
    right = (right / abs(right)) * 0.15;
  }

  interface->setMotorOutput(left, right);
}

void VelocityController::setVelocity(float left, float right) {
  leftTargetVelocity = left;
  rightTargetVelocity = right;
}

void VelocityController::setPolarVelocity(float forward, float angular) {
  setVelocity(forward - (PI * interface->getRobotRadius() * angular),
              forward + (PI * interface->getRobotRadius() * angular));
}

float VelocityController::getRampVelocity(float vMax, float vRampRate, float t) {
  return max(0., min(vMax, vRampRate * t));
}
