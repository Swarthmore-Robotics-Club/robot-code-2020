#ifndef VELOCITYCONTROLLER_H_
#define VELOCITYCONTROLLER_H_

#include "RobotController.h"
#include "PIDLoop.h"

class VelocityController : public RobotController {
public:
  VelocityController();

  virtual ~VelocityController();

  virtual void doUpdate(float t, float dt);

  void setVelocity(float left, float right);

  void setPolarVelocity(float forward, float angular);

  static float getRampVelocity(float vMax, float vRampRate, float t);

private:
  PIDLoop leftPidLoop;
  PIDLoop rightPidLoop;

  float leftTargetVelocity = 0;
  float rightTargetVelocity = 0;
};

#endif
