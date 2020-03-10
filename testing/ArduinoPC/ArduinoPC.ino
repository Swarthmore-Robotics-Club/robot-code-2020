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

char data[128];
int sumBL = 0, sumFL = 0, sumBR = 0, sumFR = 0, sumFF = 0;
volatile long leftEncoder = 0, rightEncoder = 0, prevLeftEncoder = 0, prevRightEncoder = 0;
void getSensorReadings() {
  sumBL = 0;
  sumFL = 0;
  sumBR = 0;
  sumFR = 0;
  sumFF = 0;
  for (int i = 0; i < 25; i++) {
    int bl = analogRead(SBL);
    sumBL += bl;
    int fl = analogRead(SFL);
    sumFL += fl;
    int br = analogRead(SBR);
    sumBR += br;
    int fr = analogRead(SFR);
    sumFR += fr;
    int ff = analogRead(SFF);
    sumFF += ff;
    delayMicroseconds(10);
  }
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

void setup(){
  Serial1.begin(9600);
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
}
void setMotorDirection(bool forward, bool leftMotor) {
  digitalWrite(leftMotor ? INB1 : INA1, forward ? HIGH : LOW);
  digitalWrite(leftMotor ? INB2 : INA2, forward ? LOW : HIGH);
}

void setMotors(float leftVoltage, float rightVoltage) {
  float l = (leftVoltage);
  float r = (rightVoltage);
  analogWrite(PWMB, l);
  analogWrite(PWMA, r);
  prettyPrint("Wrote l: " + (String)l + ", r: " + (String)r + "\n");
}
void prettyPrint(String s) {
  char buf[1000];
  s.toCharArray(buf, 1000);
  Serial1.write(buf);
}

bool a = true;
void loop() {
  if (digitalRead(OOS) == LOW) { setMotors(0, 0); prettyPrint("OOS\n"); delay(3000); return; }
  getSensorReadings();
  if (leftEncoder < 642) {
    setMotors(40, 40);
  } else {
    if (a) { prettyPrint("STOPPING\n"); a = false; }
    setMotors(0, 0);
  }
  prettyPrint("\tLeft: " + (String)(leftEncoder) + ", Right: " + (String)(rightEncoder) + "\n");
}
