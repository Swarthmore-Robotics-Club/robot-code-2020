#include "WallAlignController.h"

WallAlignController::WallAlignController(RobotInterface* iface) 
  : orientationPid(1e-10, 0, 0, 0),
    VelocityController(iface) {
}

WallAlignController::~WallAlignController() {

}

float WallAlignController::getDistanceDiff() {
  // todo: edge case for when neither is a good reading
  int sumBL = 0;
  int sumFL = 0;
  int sumBR = 0;
  int sumFR = 0;
  for (int i = 0; i < 25; i++) {
    sumBL += interface->getRearLeftDistance();
    sumFL += interface->getFrontLeftDistance();
    sumBR += interface->getRearRightDistance();
    sumFR += interface->getFrontRightDistance();
    delayMicroseconds(10);
  }
  if (sumBR + sumFR > sumFL + sumBL) {
    return (sumFR - sumBR) / 25.0;
  }
  return (sumBL - sumFL) / 25.0;
}

void WallAlignController::doUpdate(double t, double dt) {
  float diff = getDistanceDiff();
  float angularVelocity = orientationPid.updateError(0, diff, dt);
  if (fabs(diff) < ALIGNMENT_TOLERANCE) {
    angularVelocity = 0;
  }

  setPolarVelocity(0, angularVelocity);
  VelocityController::doUpdate(t, dt);
}
