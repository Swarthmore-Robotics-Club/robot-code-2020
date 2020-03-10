#ifndef MOTIONPROFILEFORWARDCONTROLLER_H_
#define MOTIONPROFILEFORWARDCONTROLLER_H_

#include "VelocityController.h"

class MotionProfileForwardController : public VelocityController {
public:
  MotionProfileForwardController();

  virtual ~MotionProfileForwardController();

private:
  float vMax;
  float vRampRate;
  float startTime;
  float forwardDistance;
  float encoderStartLeft;
  float encoderStartRight;
  float halfwayTime = 0;
  bool halfway = false;
};

#endif
