#include "RobotInterface.h"
#include "RobotController.h"
#include "STM32Interface.h"
#include "VelocityController.h"
#include "WallAlignController.h"

RobotInterface* robotInterface;
RobotController* currentController;

VelocityController* velocityController;
WallAlignController* wallAlignController;

unsigned long prev_time;

void setup() {
  robotInterface = new STM32Interface();
  velocityController = new VelocityController(robotInterface);
  wallAlignController = new WallAlignController(robotInterface);
//  currentController = wallAlignController;

  prev_time = micros();
  Serial2.begin(9600);
}


float sum_dt = 0;
void debug(double dt, double t) {
  sum_dt += dt;
  if (sum_dt > 0.1) {
    Serial2.print("velocity ");
    Serial2.print(robotInterface->getLeftVelocity());
    Serial2.print(" ");
    Serial2.print(robotInterface->getRightVelocity());
//    Serial2.print(" encoders ");
//    Serial2.print(robotInterface->getLeftEncoderRaw());
//    Serial2.print(" ");
//    Serial2.print(robotInterface->getRightEncoderRaw());
    Serial2.print(" positioning ");
    Serial2.print(robotInterface->getFrontLeftDistance());
    Serial2.print(" ");
    Serial2.print(robotInterface->getRearLeftDistance());
    Serial2.print(" ");
    Serial2.print(robotInterface->getFrontRightDistance());
    Serial2.print(" ");
    Serial2.print(robotInterface->getRearRightDistance());
    Serial2.println();
    sum_dt = 0;
  }
}

void loop() {
  double dt = (double) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  double t = prev_time * 1e-6;
  
  robotInterface->doUpdate(t, dt);
  if (currentController) {
    currentController->doUpdate(t, dt);
  }

  debug(dt, t);

  delay(1);
}
