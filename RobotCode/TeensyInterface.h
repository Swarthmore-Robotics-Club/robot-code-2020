#ifndef TEENSYINTERFACE_H_
#define TEENSYINTERFACE_H_

#include "RobotInterface.h"

class TeensyInterface : public RobotInterface {
  public:
    TeensyInterface();
    virtual ~TeensyInterface();
    virtual void setMotorOutput(float left, float right) override;
    virtual long getLeftEncoderRaw();
    virtual long getRightEncoderRaw();
    virtual int getFrontFrontDistance();
    virtual int getFrontLeftDistance();
    virtual int getFrontRightDistance();
    virtual int getRearLeftDistance();
    virtual int getRearRightDistance();
    virtual float getRobotRadius();
    virtual float getWheelRadius();
    virtual float getTicksPerRevolution();
    virtual PIDConstants getLeftWheelPIDConstants();
    virtual PIDConstants getRightWheelPIDConstants();
    static void encoderLeftChangeA();
    static void encoderLeftChangeB();
    static void encoderRightChangeA();
    static void encoderRightChangeB();

  private:
    static const int PIN_MOTOR_LEFT_PWM = 10;
    static const int PIN_MOTOR_RIGHT_PWM = 9;
    static const int PIN_MOTOR_LEFT_A = 8;
    static const int PIN_MOTOR_LEFT_B = 7;
    static const int PIN_MOTOR_RIGHT_A = 11;
    static const int PIN_MOTOR_RIGHT_B = 12;

    static const int PIN_ENCODER_LEFT_A = 5;
    static const int PIN_ENCODER_LEFT_B = 4;
    static const int PIN_ENCODER_RIGHT_A = 20;
    static const int PIN_ENCODER_RIGHT_B = 21;

    static const int PIN_DISTANCE_SENSOR_FRONT_LEFT = 17;
    static const int PIN_DISTANCE_SENSOR_FRONT_RIGHT = 16;
    static const int PIN_DISTANCE_SENSOR_REAR_LEFT = 15;
    static const int PIN_DISTANCE_SENSOR_REAR_RIGHT = 14;
    static const int PIN_DISTANCE_SENSOR_FRONT_FRONT = 18;

    static const int PIN_ON_OFF_SWITCH = 13;

    static long leftEncoder;
    static long rightEncoder;

    static PIDConstants leftWheelPID = (PIDConstants){0, 0, 0, 0};
    static PIDConstants rightWheelPID = (PIDConstants){0, 0, 0, 0};
};

#endif
