class PIDLoop {
  public:
    PIDLoop(float kP1, float kI1, float kD1) {
      kP = kP1;
      kI = kI1;
      kD = kD1;
    }
    float updateError(float error, unsigned long dt) {
      float oldError = pError;
      pError = error;
      if (error == 0) {
        iError = 0;
      } else {
        iError += (error * dt);
      }
      dError = (error - oldError) / dt;
      return kP * pError + kI * iError + kD * dError;
    }
    void wipe() {
      pError = 0;
      iError = 0;
      dError = 0;
    }
  private:
    float pError = 0, iError = 0, dError = 0;
    float kP, kI, kD;
};

enum State { ORIENT, DETERMINE, FORWARD, TURN, CHILL };
