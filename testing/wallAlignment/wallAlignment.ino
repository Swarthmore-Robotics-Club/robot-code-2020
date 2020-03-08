#define PWMA 9 // right motor
#define PWMB 10 // left motor
#define INA1 8
#define INA2 7
#define INB1 11
#define INB2 12

#define SBR 14
#define SBL 15
#define SFR 18
#define SFL 17

/* PID loop constants */
#define KP 0.001 //0.0015
#define KI 0.0000
#define KD 0

/* EWMA */
#define STARTINGAVERAGE 0
#define BETA .875

/* */
#define MAX_SPEED 60
#define WHEEL_BASE_LENGTH 0.085
#define WHEEL_RADIUS 0.0236
#define TOLERANCE 5
#define MOTOR_CONSTANT 1

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
        iError += (error * dt);
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
    float pError, iError, dError;
    float kP, kI, kD;
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
int quit = 1;

void setup() {
  Serial.begin(9600);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(SFR, INPUT);
  currentTime = millis();
  pid->wipe();
  set_left_motor_direction(1);
  set_right_motor_direction(1);
}

void loop() {
  if (quit > 0)  { set_motors(0, 0); delay(100000); return; }
  unsigned long newTime = millis();
  int dt = newTime - currentTime;
  currentTime = newTime;

  int BR = analogRead(SBR);
  int BL = analogRead(SBL);
  int FR = analogRead(SFR);
  int FL = analogRead(SFL);
  float leftDiff = BL - FL;

  float average = ewma->addReading(leftDiff);
  float angular_velocity = pid->updateError(average, dt);
  float forward_velocity = MAX_SPEED / (abs(angular_velocity) + 1);
  sprintf(data, "angular_vel: %f, average: %f, BL: %d, FL: %d\n", angular_velocity, average, BL, FL);
  Serial.print(data);
  
  if (abs(average) < TOLERANCE) {
    set_motors(0, 0);
  } else {
    float left, right;
    unicycle_to_differential_drive(&left, &right, 0, angular_velocity, WHEEL_RADIUS, WHEEL_BASE_LENGTH);
    set_left_motor_direction(left >= 0);
    set_right_motor_direction(right >= 0);
    set_motors(abs(left) * MOTOR_CONSTANT, abs(right) * MOTOR_CONSTANT);
    sprintf(data, "\tleft: %f, right: %f\n", left, right);
    Serial.print(data);
  }
  delay(100);
}

void set_left_motor_direction(int forward) {
  if (forward) {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
  } else {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
  }
}

void set_right_motor_direction(int forward) {
  if (forward) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
  } else {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
  }
}

void set_motors(float left, float right) {
  float l, r;
  if (right == 0 && left == 0) {
    l = 0;
    r = 0;
  } else if (left < right) {
    l = left / right;
    r = 1.0;
  } else {
    l = 1.0;
    r = right / left;
  }
  analogWrite(PWMA, r * MAX_SPEED);
  analogWrite(PWMB, l * MAX_SPEED);
  sprintf(data, "\tWrote %f to left motor, %f to right\n", l * MAX_SPEED, r * MAX_SPEED);
  Serial.print(data);
}
