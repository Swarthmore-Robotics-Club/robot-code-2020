/* Right motor */
#define PWMA 9
/* Left motor */
#define PWMB 10

/* Motor directions */
#define INA1 8
#define INA2 7
#define INB1 11
#define INB2 12

/* Sensors */
#define SBR 14
#define SBL 15
#define SFR 16
#define SFL 17
#define SFF 18

/* Encoders */
#define ERA 20
#define ERB 21
#define ELA 5
#define ELB 4

/* PID loop constants */
#define KP 0.0015
#define KI 0
#define KD 0.000375

/* Physical realities */
#define MAX_VOLTAGE 255
#define MIN_VOLTAGE 28
#define MAX_SPEED .8 // meters per second
#define MAX_RPM 430
#define WHEEL_BASE_LENGTH 0.101 // meters
#define WHEEL_RADIUS 0.0215 // meters

/* Fake realities */
#define TOLERANCE 20
#define DESIRED_MAX_VOLTAGE 60
#define TOO_LOW_VOLTAGE 10
#define CAP_VOLTAGE(arg1) (arg1 >= DESIRED_MAX_VOLTAGE ? DESIRED_MAX_VOLTAGE : (arg1 < TOO_LOW_VOLTAGE ? 0 : max(arg1, MIN_VOLTAGE)))
#define MAX_ANGULAR_VELOCITY 32

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
    float pError, iError, dError;
    float kP, kI, kD;
};

char data[64];
PIDLoop* pid = new PIDLoop(KP, KI, KD);
unsigned long currentTime;

void setup() {
  Serial.begin(9600);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(SFF, INPUT);
  pinMode(SFR, INPUT);
  pinMode(SFL, INPUT);
  pinMode(SBL, INPUT);
  pinMode(SBR, INPUT);
  pinMode(ERA, INPUT);
  pinMode(ERB, INPUT);
  pinMode(ELB, INPUT);
  pinMode(ELA, INPUT);
  currentTime = millis();
  pid->wipe();
  set_left_motor_direction(true);
  set_right_motor_direction(true);
  delay(4000);
}

void loop() {
  int quit = 1;
  if (quit > 1)  { set_motors(0, 0); delay(100000); return; }
  unsigned long newTime = millis();
  unsigned long dt = (newTime - currentTime);
  currentTime = newTime;
  float average = getDiff();
  float angular_velocity = pid->updateError(average, dt);
  float forward_velocity = MAX_SPEED / (abs(angular_velocity) + 1);
  
  float left, right;
  if (abs(average) < TOLERANCE) {
    left = 0;
    right = 0;
  } else {
    unicycle_to_differential_drive(&left, &right, 0, angular_velocity, WHEEL_RADIUS, WHEEL_BASE_LENGTH);
    set_left_motor_direction(left >= 0);
    set_right_motor_direction(right >= 0);
  }
  set_motors(abs(left), abs(right));
  String d = "\taverage: " + (String)(average) + ", angular_vel: " + (String)(angular_velocity) + ", left: " + (String)(left) + ", right: " + (String)(right) + ", dt: " + (String)(dt) + "\n";
  Serial.print(d);
}

/* Use the sensors that are closer to a wall, more accurate readings */
float getDiff() {
  int sumBL = 0;
  int sumFL = 0;
  int sumBR = 0;
  int sumFR = 0;
  for (int i = 0; i < 25; i++) {
    sumBL += analogRead(SBL);
    sumFL += analogRead(SFL);
    sumBR += analogRead(SBR);
    sumFR += analogRead(SFR);
    delay(1);
  }
  if (sumBR + sumFR > sumFL + sumBL) {
    return (sumFR - sumBR) / 25.0;
  }
  return (sumBL - sumFL) / 25.0;
}

void set_left_motor_direction(bool forward) {
  if (forward) {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
  } else {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
  }
}

void set_right_motor_direction(bool forward) {
  if (forward) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
  } else {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
  }
}

const float thirtyOverPi = 30 / PI;

void set_motors(float left, float right) {
  float l = CAP_VOLTAGE(map(left * thirtyOverPi, 0, MAX_ANGULAR_VELOCITY, 0, MAX_VOLTAGE));
  float r = CAP_VOLTAGE(map(right * thirtyOverPi, 0, MAX_ANGULAR_VELOCITY, 0, MAX_VOLTAGE));
  analogWrite(PWMB, l);
  analogWrite(PWMA, r);
  Serial.print("Wrote l: " + (String)l + ", r: " + (String)r + " \n");
}
