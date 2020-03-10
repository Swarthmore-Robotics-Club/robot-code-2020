#ifndef WALLALIGNCONTROLLER_H_
#define WALLALIGNCONTROLLER_H_

#include "VelocityController.h"
#include "PIDLoop.h"

class WallAlignController : public VelocityController {
public:
  WallAlignController(RobotInterface* iface);

  virtual ~WallAlignController();

  virtual void doUpdate(double t, double dt);
protected:
  float getDistanceDiff();

private:
  PIDLoop orientationPid;

  const float ALIGNMENT_TOLERANCE = 15.0;
};

#endif
