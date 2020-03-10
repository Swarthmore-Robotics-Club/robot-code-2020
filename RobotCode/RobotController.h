#ifndef ROBOTCONTROLLER_H_
#define ROBOTCONTROLLER_H_

#include "RobotInterface.h"

class RobotController {
public:
  virtual void doUpdate(float t, float dt);

protected:
  RobotInterface* interface;
};

#endif
