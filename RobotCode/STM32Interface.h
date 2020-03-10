#ifndef STM32INTERFACE_H_
#define STM32INTERFACE_H_

#include "RobotInterface.h"

class STM32Interface : public RobotInterface {
public:
  STM32Interface();

  virtual ~STM32Interface();

  virtual void setMotorOutput(float left, float right) override;

  virtual long getLeftEncoderRaw();

  virtual long getRightEncoderRaw();

  virtual float getFrontLeftDistance();

  virtual float getFrontRightDistance();

  virtual float getRearLeftDistance();

  virtual float getRearRightDistance();

  virtual float getRobotRadius();

  virtual float getWheelRadius();

  virtual float getTicksPerRevolution();

  static void encoderLeftChangeA();

  static void encoderLeftChangeB();

  static void encoderRightChangeA();

  static void encoderRightChangeB();

private:
// everything below needs to be static because we are attaching interrupts that use these
// this is hacky; feel free to find a better way if you want
  static const int PIN_MOTOR_LEFT_PWM = PB0;
  static const int PIN_MOTOR_RIGHT_PWM = PB1;
  static const int PIN_MOTOR_LEFT_A = PB5;
  static const int PIN_MOTOR_LEFT_B = PA8;
  static const int PIN_MOTOR_RIGHT_A = PB7;
  static const int PIN_MOTOR_RIGHT_B = PB6;

  static const int PIN_ENCODER_LEFT_A = PA6;
  static const int PIN_ENCODER_LEFT_B = PA4;
  static const int PIN_ENCODER_RIGHT_A = PA9;
  static const int PIN_ENCODER_RIGHT_B = PA10;

  static const int PIN_DISTANCE_SENSOR_FRONT_LEFT = PA0;
  static const int PIN_DISTANCE_SENSOR_FRONT_RIGHT = PA1;
  static const int PIN_DISTANCE_SENSOR_REAR_LEFT = PA7;
  static const int PIN_DISTANCE_SENSOR_REAR_RIGHT = PA5;
  static const int PIN_DISTANCE_SENSOR_FRONT = PA11;

  static long leftEncoder;
  static long rightEncoder;

  bool initialized = false;
};

#endif
