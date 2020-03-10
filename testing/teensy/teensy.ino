#include "utils.cpp"

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

/* On off switch */
#define OOS 13

/* Physical robot realities */
#define BAUD_RATE 9600
#define MAX_VOLTAGE 255
#define MIN_VOLTAGE 28
#define MAX_SPEED .8 // meters per second
#define MAX_RPM 430
#define WHEEL_BASE_LENGTH 0.101 // meters
#define WHEEL_RADIUS 0.0215 // meters
#define WHEEL_CIRCUMFERENCE (2 * WHEEL_RADIUS * PI) // meters
#define TICKS_PER_ROTATION 900

/* Physical maze realities */
#define LENGTH_OF_CELL 0.18 // meters
#define ROTATIONS_PER_CELL_LENGTH (LENGTH_OF_CELL / WHEEL_CIRCUMFERENCE)
#define TICKS_PER_CELL (ROTATIONS_PER_CELL_LENGTH * TICKS_PER_ROTATION)

/* Fake realities */
#define DESIRED_MAX_VOLTAGE 60
#define TOO_LOW_VOLTAGE 10
#define CAP_VOLTAGE(arg1) (arg1 >= DESIRED_MAX_VOLTAGE ? DESIRED_MAX_VOLTAGE : (arg1 < TOO_LOW_VOLTAGE ? 0 : max(arg1, MIN_VOLTAGE)))
#define NUM_TIMES_TO_READ_SENSORS 25.0

/* Global variables */
unsigned long currentTime;
volatile long leftEncoder = 0, rightEncoder = 0, prevLeftEncoder = 0, prevRightEncoder = 0;
float sumBL = 0, sumFL = 0, sumBR = 0, sumFR = 0, sumFF = 0;
State occident = ORIENT;

void prettyPrint(String s) {
  char buf[1000];
  s.toCharArray(buf, 1000);
  Serial1.write(buf);
  Serial.println(s);
}

/* ISRs for encoders */
void encoderLeftAChange() {
  leftEncoder -= (1 - 2*digitalRead(ELA)) * (1 - 2*digitalRead(ELB));
}

void encoderLeftBChange() {
  leftEncoder += (1 - 2*digitalRead(ELB)) *  (1 - 2*digitalRead(ELA));
}

void encoderRightAChange() {
  rightEncoder += (1 - 2*digitalRead(ERA)) * (1 - 2*digitalRead(ERB));
}

void encoderRightBChange() {
  rightEncoder -= (1 - 2*digitalRead(ERB)) *  (1 - 2*digitalRead(ERA));
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);
  
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
  pinMode(OOS, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ERA), encoderRightAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ERB), encoderRightBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELA), encoderLeftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELB), encoderLeftBChange, CHANGE);
  
  setMotorDirection(true, true);
  setMotorDirection(true, false);
  currentTime = millis();
  delay(10);
}

void loop() {
  /* Handle timing */
  unsigned long newTime = millis();
  unsigned long dt = (newTime - currentTime);
  currentTime = newTime;

  /* Check off switch */
  if (digitalRead(OOS) == LOW) { setMotors(0, 0); prettyPrint("OOS\n"); delay(3000); return; }

  /* Everyone needs sensors - function creates total delay of NUM_TIMES_TO_READ_SENSORS * 10 microseconds */
  getSensorReadings();

  switch(occident) {
    case ORIENT:
      orientYoSelfBeforeYouOccidentYoSelf(dt); break;
    case DETERMINE:
      deTerminator(); break;
    case TURN:
      dreidal(); break;
    case FORWARD:
      toInfinityAndBeyond(); break;
    case CHILL:
      setMotors(0, 0); delay(1000); break;
    default:
      prettyPrint("Defaulted!\n"); prettyPrint((String)occident); delay(1000);
  }
}

void getSensorReadings() {
  sumBL = 0;
  sumFL = 0;
  sumBR = 0;
  sumFR = 0;
  sumFF = 0;
  for (int i = 0; i < NUM_TIMES_TO_READ_SENSORS; i++) {
    sumBL += analogRead(SBL);
    sumFL += analogRead(SFL);
    sumBR += analogRead(SBR);
    sumFR += analogRead(SFR);
    sumFF += analogRead(SFF);
    delayMicroseconds(10);
  }
}

void setMotorDirection(bool forward, bool leftMotor) {
  digitalWrite(leftMotor ? INB1 : INA1, forward ? HIGH : LOW);
  digitalWrite(leftMotor ? INB2 : INA2, forward ? LOW : HIGH);
}

void setMotors(float leftVoltage, float rightVoltage) {
  float l = CAP_VOLTAGE(leftVoltage);
  float r = CAP_VOLTAGE(rightVoltage);
  analogWrite(PWMB, l);
  analogWrite(PWMA, r);
  prettyPrint("Wrote l: " + (String)l + ", r: " + (String)r + "\n");
}
