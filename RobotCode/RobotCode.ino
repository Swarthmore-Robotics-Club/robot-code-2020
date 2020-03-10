#include "RobotInterface.h"
#include "RobotController.h"
#include "STM32Interface.h"

RobotInterface* robotInterface;
RobotController* currentController;

unsigned long prev_time;

void setup() {
  robotInterface = new STM32Interface();
  currentController = NULL;

  prev_time = micros();
  Serial2.begin(9600);
}

void loop() {
  float dt = (float) (micros() - prev_time) * 1e-6;
  prev_time = micros();
  float t = prev_time * 1e-6;
  
  robotInterface->doUpdate(t, dt);
  if (currentController) {
    currentController->doUpdate(t, dt);
  }

  delay(1);
}
