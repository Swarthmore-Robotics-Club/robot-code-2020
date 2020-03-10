#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

class RobotInterface {
protected:
  RobotInterface();
  virtual ~RobotInterface();

public:
// need to implement in base class
  virtual void setMotorOutput(float left, float right) = 0;
  virtual long getLeftEncoderRaw() = 0;
  virtual long getRightEncoderRaw() = 0;
  virtual float getFrontLeftDistance() = 0;
  virtual float getFrontRightDistance() = 0;
  virtual float getRearLeftDistance() = 0;
  virtual float getRearRightDistance() = 0;
  virtual float getRobotRadius() = 0;
  virtual float getWheelRadius() = 0;
  virtual float getTicksPerRevolution() = 0;

// can implement in base class, but be sure to call super
  virtual void doUpdate(float t, float dt);

// derived quantities
  float getLeftEncoder();

  float getRightEncoder();

  float getLeftVelocity();

  float getRightVelocity();

  float getDistancePerTick();

private:
  // exponential moving average coefficient
  const float velocityEmaCoefficient = 0.9;

  // encoder information for velocity
  float prevLeftEncoder = 0;
  float prevRightEncoder = 0;

  // timer used to only recompute velocity every velocityRecomputeEvery seconds
  float velocityRecomputeTimer = 0;
  float velocityRecomputeEvery = 0.01;

  // velocity information
  float leftVelocity = 0;
  float rightVelocity = 0;
};

#endif
