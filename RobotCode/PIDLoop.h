#ifndef PIDLOOP_H_
#define PIDLOOP_H_

class PIDLoop {
  public:
    PIDLoop(float kP1, float kI1, float kD1, float kF1);
    float updateError(float target, float actual, double dt);
    void wipe();

  private:
    float pError, iError, dError;
    float kP, kI, kD, kF;
};

#endif
