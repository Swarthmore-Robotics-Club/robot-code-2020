#define PWMA 9
#define PWMB 10
#define INA1 8
#define INA2 7
#define INB1 11
#define INB2 12

#define SBR 14
#define SBL 15
#define SFR 18
#define SFL 17

#define KP 0.015
#define KI 0
#define KD 0
#define STARTINGAVERAGE 0
#define BETA .875

class PIDLoop {
  public:
    PIDLoop(float kP1, float kI1, float kD1) {
      kP = kP1;
      kI = kI1;
      kD = kD1;
    }
    float updateError(float error, int dt) {
      float oldError = pError;
      pError = error;
      if (error == 0) {
        iError = 0;
      } else {
        iError = iError + (error * dt);
      }
      dError = (error - oldError) / 2;
      return kP * pError + kI * iError + kD * dError;
    }
    void wipe() {
      pError = 0;
      iError = 0;
      dError = 0;
    }
  private:
    int kP, kI, kD;
    float pError, iError, dError;
};

class EstimatedWeightedMovingAverage {
  public:
    EstimatedWeightedMovingAverage(float b, float startingAverage) {
      beta = b;
      average = startingAverage;
    }
    float addReading(float value) {
      average = (beta * average) + ((1 - beta) * value);
      return average;
    }
  private:
    float beta, average;
};

char data[64];
EstimatedWeightedMovingAverage* ewma = new EstimatedWeightedMovingAverage(BETA, STARTINGAVERAGE);
PIDLoop* pid = new PIDLoop(KP, KI, KD);
unsigned long currentTime;

void setup() {
  Serial.begin(9600);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(SFR, INPUT);
  currentTime = millis();
}

void loop() {
  unsigned long newTime = millis();
  unsigned long dt = currentTime - newTime;
  currentTime = newTime;
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);

  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  int BR = analogRead(SBR);
  int BL = analogRead(SBL);
  int FR = analogRead(SFR);
  int FL = analogRead(SFL);
  float leftDiff = FL - BL;

  float average = ewma->addReading(leftDiff);
  
  sprintf(data, "%f  FL: %d  BL: %d\n\t%f\n\t%lx\n", leftDiff, FL, BL, average, dt);
  Serial.print(data);
 
  delay(150);
}
