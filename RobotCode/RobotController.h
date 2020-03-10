#ifndef ROBOTCONTROLLER_H_
#define ROBOTCONTROLLER_H_

#include "RobotInterface.h"

class RobotController {
public:
  virtual void doUpdate(double t, double dt) = 0;

protected:
  RobotController(RobotInterface* iface);
  virtual ~RobotController();

  RobotInterface* interface;
};

#endif
