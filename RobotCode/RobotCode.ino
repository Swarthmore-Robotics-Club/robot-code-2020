#include "RobotInterface.h"
#include "RobotController.h"
#include "TeensyInterface.h"
#include "VelocityController.h"
#include "WallAlignController.h"

RobotInterface* robotInterface;
RobotController* currentController;

VelocityController* velocityController;
WallAlignController* wallAlignController;

unsigned long prev_time;

void setup() {
  robotInterface = new TeensyInterface();
  velocityController = new VelocityController(robotInterface);
  wallAlignController = new WallAlignController(robotInterface);
  currentController = velocityController;

  prev_time = micros();
}

void loop() {
  double dt = (double) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  double t = prev_time * 1e-6;
  
  robotInterface->doUpdate(t, dt);
  if (currentController) {
    currentController->doUpdate(t, dt);
  }

  delay(1);
}
