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
  currentController = wallAlignController;


  pinMode(13, INPUT_PULLUP);
  prev_time = micros();
  Serial.begin(9600);
}

void loop() {
  double dt = (double) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  double t = prev_time * 1e-6;
  if (digitalRead(13) == LOW) { robotInterface->setMotorOutput(0,0); Serial.println("OOS"); delay(3000); return; }
  
  robotInterface->doUpdate(t, dt);
  if (currentController) {
    currentController->doUpdate(t, dt);
  }

  delay(1);
}
