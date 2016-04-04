/*
  Jason - dropoff, pickup
  Seran  - look
  Matt - placetesseract, position, gohome, return
  Eric - check, move
*/

#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>

//DEBUGGERS -> uncomment to debug
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS

//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool TurnRight = true;
int AnyUse;
unsigned pickedUp;
bool Start = true;

//Hall Sensor Stuff
#define NOFIELDGRIP 513L
#define NOFIELDRGT 512L
#define NOFIELDLFT 503L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  
int currentHallRead;
int lastHallRead;

//Mechanical Information
unsigned WheelPerimeter = (69.85 * PI) / 10; //perimeter of wheel in cm
unsigned ForwardSpeed = 1650; //speed of robot while looking in mode 1
unsigned Stop = 1500;

//Line Tracker Stuff
unsigned LineTrackerData = 0;
unsigned GripLightData = 0;
unsigned GripLightDark = 0;
bool HitBlack = false;
int HitBlackCount = 0;
int HitBlackTarget = 3;

//Data variables
unsigned long HallSensorValue = 0;
unsigned long UltrasonicDistance = 0;
unsigned int timer;
unsigned long timerStart;


Servo LftMtr;
Servo ArmBend;    //out -> folded 0->180
Servo ArmBase;    //folded->out  37-180
Servo RgtMtr;
Servo Grip;       //170 closed, 100 open
Servo Wrist;      //0 min folded up, 50 straight out, 180 folded down
I2CEncoder LftEncdr;
I2CEncoder RgtEncdr;
I2CEncoder ArmBaseEncdr;
I2CEncoder ArmBendEncdr;

//Mode Selector Variables
unsigned int ModeIndex = 0;
unsigned int ModeIndicator[6] = {
  0x00, //Default Mode (Mode 0)
  0x00FF, //Mode 1
  0x0F0F, //Mode 2
  0x3333, //Calibrate Line Tracker to Dark
  0xAAAA, //Calibrate Motors (might not need)
  0xFFFF
};

//pins
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 6;
const int ArmBendPin = 7;
const int GripPin = 10;
const int WristPin = 11;
const int HallRgt = A0;
const int HallLft = A1;
const int GripLight = A2;
const int HallGrip = A3;
//*****cannot plug into A4 or A5
const int UltrasonicPing = 2;//data return in 3
const int UltrasonicPingSide = 8;//data return in 9

// variables
unsigned int MotorSpeed;
unsigned int LftMotorSpeed;
unsigned int RgtMotorSpeed;
unsigned int LftMotorPos;
unsigned int RgtMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;
long lftEncoderCounter;
long rgtEncoderCounter;

// Tracking Variables
unsigned long CourseWidth = 240; //course width in cm, has to be set prior to running
unsigned long XPos = 0;
long RawLftPrv = 0;
long RawRgtPrv = 0;
const double CE = 637;//pulses per revolution
const double CF = ((3.14159 * 69.85) / CE); //Conversion factor, traslates encoder pulses to linear displacement
double DelLft = 0;
double DelRgt = 0;
double DelDsp = 0;
long TotalDsp = 0;
double SvdDsp = 0;
double Dsp = 0;
double OrTheta = 0;
double PrvOrTheta = 0;
double dTheta = 0;
double PolTheta = 0;
double FindTheta = 0;
double PickUpTheta = 0;
double XPstn = 0;
double dXPstn = 0;
double YPstn = 0;
double dYPstn = 0;
double ThetaBuffer = 2;
double DspBuffer = 10;
int StepIndex = 1;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  pinMode(GripPin, OUTPUT);
  Grip.attach(GripPin);
  Grip.write(100);

  pinMode(WristPin, OUTPUT);
  Wrist.attach(WristPin);
  Wrist.write(80);

  // Set up encoders DO NOT CHANGE ORDER
  RgtEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward

  LftEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward

  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  ArmBase.write(55);

  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBend.write(150);

  pinMode(7, INPUT);

  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicPing + 1, INPUT);
  pinMode (UltrasonicPingSide, OUTPUT);
  pinMode(UltrasonicPingSide + 1, INPUT);
}

void loop() {
  //***************************stuff running through every time
  DebuggerModule();
  timer = millis() / 1000; //time in seconds

  if (timer % 30 < 1) {
    Serial.print("time = ");
    Serial.println(timer);
  }
  if (timer >= 240) {  //4 min time limit
    GoHome(1);       // **********************************************uncomment this!!!!!!!!!!!
    ModeIndex = 0;
  }
  if (Start) {       //only runs this until mode started, 1 sec delay, must reset to change mode
    if (digitalRead(13)) {   //switch 1 up = stay in 0, down = start mode 1 or 2
      Start = false;
      timerStart = millis();
      if (digitalRead(12)) ModeIndex = 2; // switch 3 and 1 on (down)
      else  ModeIndex = 1; // switch 3 off (up), 1 on (down)
      delay(1000);
    }
    else ModeIndex = 0; // switch 1 off(up), select mode then turn switch 1 on(down) when want to start
  }
  if (!(digitalRead(13))) ModeIndex = 0;

  switch (ModeIndex) {
    case 0 : /***********sitting around waiting, use this mode to test stuff, then clear*/ Serial.println("In Mode 0");
      PickUp(1);
      delay(5000);
      break;

    case 1: /**********************************mode 1 base = look */ Serial.println("In mode 1");

      if (!(timer % 3)) Ping(UltrasonicPing);
      Serial.print("d =   ");
      Serial.println(UltrasonicDistance);
      if ((UltrasonicDistance < 18) && (UltrasonicDistance != 0)) {
        if ((analogRead(HallLft) - NOFIELDLFT > 6) || (analogRead(HallLft) - NOFIELDLFT < -6) || (analogRead(HallRgt) - NOFIELDRGT > 6) || (analogRead(HallRgt) - NOFIELDRGT < -6)) {
          Serial.println("moving");
          RgtMtr.writeMicroseconds(ForwardSpeed);
          LftMtr.writeMicroseconds(ForwardSpeed);
        }
        else {
          Serial.println("detected tesserract******************");
          LftMtr.writeMicroseconds(Stop);
          RgtMtr.writeMicroseconds(Stop);
          PickUp();
        }
      }
      else { //if reaches end of course width, turn right then left
        Serial.print("turning  ");
        Serial.println(UltrasonicDistance);
        if (TurnRight) {
          RgtMtr.writeMicroseconds(Stop);
          LftMtr.writeMicroseconds(Stop);
          delay(100);
          AnyUse = (LftEncdr.getRawPosition() + 980 );
          while ((LftEncdr.getRawPosition() < AnyUse)) {
            LftMtr.writeMicroseconds(1600);
          }
          LftMtr.writeMicroseconds(1500);
          delay(3000);
        }
        else {
          RgtMtr.writeMicroseconds(Stop);
          LftMtr.writeMicroseconds(Stop);
          delay(300);
          AnyUse = (RgtEncdr.getRawPosition() + 980);
          while ((RgtEncdr.getRawPosition() < AnyUse)) {
            RgtMtr.writeMicroseconds(1600);
          }
          RgtMtr.writeMicroseconds(1500);
          delay(3000);
        }
        delay(5000);
        Ping(UltrasonicPing);
        TurnRight = !TurnRight;
      }
      break;

    case 2:  /********************mode 2 base = check   */ Serial.println("In mode 2");
      //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
      // Robo --> back and forth scanning motion
      lastHallRead = analogRead(HallGrip);
      lftEncoderCounter = LftEncdr.getRawPosition();
      rgtEncoderCounter = RgtEncdr.getRawPosition();
      ArmBase.write(100); // 37 - 179 folded to out
      ArmBend.write(110); // 0 -180 out to folded
      LftMtr.writeMicroseconds(1650);
      for (lftEncoderCounter; lftEncoderCounter < 40; lftEncoderCounter++) {
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        Serial.print("Left Encoder Forward: ");
        Serial.println(lftEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      LftMtr.writeMicroseconds(1500);
      for (lftEncoderCounter; lftEncoderCounter > 0; lftEncoderCounter--) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Left Encoder Backward: ");
        Serial.println(lftEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      LftMtr.writeMicroseconds(1500);
      delay(200);
      RgtMtr.writeMicroseconds(1650);
      for (rgtEncoderCounter; rgtEncoderCounter < 40; rgtEncoderCounter++) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Right Encoder Forward: ");
        Serial.println(rgtEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      RgtMtr.writeMicroseconds(1500);
      for (rgtEncoderCounter; rgtEncoderCounter > 0; rgtEncoderCounter--) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Right Encoder Backward: ");
        Serial.println(rgtEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      RgtMtr.writeMicroseconds(1500);
      delay(200);

      break;

    case 3:

      break;
      //etc. add as needed
  }
}

void DebuggerModule() {
  //Debugger module -> all debugger code can go here

#ifdef DEBUG_HALL_SENSOR
  Serial.println((analogRead(HallLft) - NOFIELDLFT) * TOMILLIGAUSS / 1000);
  Serial.println((analogRead(HallRgt) - NOFIELDRGT) * TOMILLIGAUSS / 1000);
#endif

#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(UltrasonicDistance * 58, DEC);
  Serial.print(", cm's: ");
  Serial.println(UltrasonicDistance);
#endif

#ifdef DEBUG_LINE_TRACKER
  Serial.print("Light Level: ");
  Serial.println(GripLightData, DEC);
#endif

#ifdef DEBUG_ENCODERS
  LftMotorPos = LftEncdr.getRawPosition();
  RgtMotorPos = RgtEncdr.getRawPosition();
  Serial.print("Encoders L: ");
  Serial.print(LftMotorPos);
  Serial.print(", R: ");
  Serial.println(RgtMotorPos);
#endif
}

//any time functions
void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) * 1.1 / 58);//returns in cm
  return;
}

void ReadLineTracker() {
  GripLightData = analogRead(GripLight);
  return;
}

//Mode 1
void TrackPosition() {
  if (EnableIncrement == false && LftMtr.read() <= 270) {
    EnableIncrement = true;
  }
  else if (EnableIncrement == false && LftMtr.read() > 270) {
    XPos += WheelPerimeter;
    EnableIncrement = false;
  }
  return;
}

void PickUp(int i) {  //left = 1, right = 0
  //robot has deteced tesseract and uses arm to pick it up, after picked up runs 'GoHome'
  Serial.println(analogRead(HallGrip) - NOFIELDGRIP);
  switch (i) {
    case 0:
      LftMtr.writeMicroseconds(1450);
      RgtMtr.writeMicroseconds(1400);
      delay(1000);
      LftMtr.writeMicroseconds(Stop);
      RgtMtr.writeMicroseconds(Stop);
      delay(200);
      LftMtr.writeMicroseconds(1600);
      RgtMtr.writeMicroseconds(1600);
      delay(1000);
      LftMtr.writeMicroseconds(Stop);
      RgtMtr.writeMicroseconds(Stop);
      i = 3;
      break;
    case 1:
      i = 3;
      break;
    case 3:
      delay(500);
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1350);
      delay(1500);
      LftMtr.writeMicroseconds(Stop);
      RgtMtr.writeMicroseconds(Stop);
      i = 9;
  }
  Grip.write(100);     //picking up tesseract
  Wrist.write(48);
  ArmBend.write(130);
  delay(1000);
  ArmBase.write(110);
  delay(1000);
  ArmBase.write(125);
  delay(1000);
  Grip.write(170);
  delay(1000);
  ArmBase.write(90);
  delay(1000);
  Serial.println(analogRead(HallGrip) - NOFIELDGRIP);
  if (!((analogRead(HallGrip) - NOFIELDGRIP) > 5 || ((analogRead(HallGrip) - NOFIELDGRIP) < -5))) { //drop tesseract if not magnetic/missed tesseract
    ArmBend.write(20);
    delay(1000);
    Grip.write(100);
    delay(500);
    for (int j = 20; j <= 130; j += 5) {
      ArmBend.write(j);
      delay(100);
    }
  }
  else {
    Serial.println("got tesseract");
    GoHome(0);
  }
  return;
}

void Position() {
  // PickUpTheta, FindTheta, SvdRgtEncdr, SvdLftEncdr

  // Distance travelled
  DelRgt = (CF * ((RgtEncdr.getRawPosition()) - RawRgtPrv)); // Instantaneous Distance traveled by right Wheel
  DelLft = CF * ((LftEncdr.getRawPosition() - RawLftPrv)); // Instantaneous Distnace traveled by left wheel
  DelDsp = (DelRgt + DelLft) / 2; //Instantaneous Distance traveled by the centerpoint of the robot
  Dsp = Dsp + DelDsp; //Current Displacement
  Serial.print("Displacement: ");
  Serial.println(Dsp);
  dTheta = ((DelRgt - DelLft) / 109) * (180 / PI); // Change in orientation, taking starting postion as Theta = 0
  OrTheta = OrTheta + dTheta; //Orientation of robot
  OrTheta = (int)OrTheta % 360; //If the magnitude of the orientation is greater than 360
  Serial.print("Orientation Theta: ");
  Serial.println(OrTheta);
  dXPstn = DelDsp * cos(OrTheta * PI / 180);
  dYPstn = DelDsp * sin(OrTheta * PI / 180);
  XPstn = XPstn + dXPstn;
  YPstn = YPstn + dYPstn;
  Serial.print("X: ");//X coordinates of the robot (right is positive)
  Serial.print(XPstn);
  Serial.print( "Y: ");//Y coordinates of the robot (up is positive)
  PolTheta = atan(YPstn / XPstn * 180 / PI); //The polar angle of the position of the robot
  RawLftPrv = LftEncdr.getRawPosition();
  RawRgtPrv = RgtEncdr.getRawPosition();
  PrvOrTheta = OrTheta;
  return;
}

void GoHome(int done) {
  //robot calculates and saves position and returns to base after tesseract picked up,
  Position();
  for (int i = 0; i > 0; i++) {
    SvdDsp = Dsp;
  }
  while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta - 5))) {
    Serial.println("Alinging Bot...");
    LftMtr.writeMicroseconds(1500);
    RgtMtr.writeMicroseconds(1300);
    Position();
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  while (Dsp > 10) {
    Serial.println("Moving towards origin...");
    LftMtr.writeMicroseconds(2000);
    RgtMtr.writeMicroseconds(2000);
    Position();
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  if (done == 1) ModeIndex = 0;
  else PlaceTesseract();
  return;
}

void Return() {
  /*
    robot is at start and has already picked up a tesseract, return to last position where tesseract was picked up, continue with 'Look'
    This operates in the following steps:

    The following uses polar coordinates in R^2

    First conditional: rotates the robot until its angle matches the saved angle prior to returning to home
    therefore: if( current angle is less than saved angle and if its distance from the home position is less than the distance from the saved position)
    action: *robot will turn*

    Second conditional: robot will go to the saved point once it faces the correct direction
    therefore: if the angle is greater or = to the saved angle (the robot should never surpass the angle by too much) and if its distance from
                    the home position is less than the distance from the saved position
    action: *robot moves forward*

    Third conditional: robot calls Look() once returning to the correct position
    therefore: if the angle is greater or = to the saved angle and if its distance from the home position is greater than or equal to the saved position distance

  */
  Position();
  if (((OrTheta < (FindTheta - ThetaBuffer)) || OrTheta > (FindTheta + ThetaBuffer)) && ((Dsp < (SvdDsp - DspBuffer)) || Dsp > (SvdDsp + DspBuffer)))
  {
    LftMotorSpeed = 1400;
    RgtMotorSpeed = 1600;
  }
  else if (((OrTheta > (FindTheta - ThetaBuffer)) || OrTheta < (FindTheta + ThetaBuffer)) && ((Dsp < (SvdDsp - DspBuffer)) || Dsp > (SvdDsp + DspBuffer)))
  {
    LftMotorSpeed = MotorSpeed + LeftMotorOffset;
    RgtMotorSpeed = MotorSpeed + RightMotorOffset;
  }
  else if (((OrTheta > (FindTheta - ThetaBuffer)) || OrTheta < (FindTheta + ThetaBuffer)) && ((Dsp > (SvdDsp - DspBuffer)) || Dsp < (SvdDsp + DspBuffer)))
  {
    //switch control signal to go back to Look();
  }
  return;
}

void PlaceTesseract() {
  /*
    1. extend arm into scan mode
    2. orient robot to be at 200 degree orientation
    3. rotate counter clockwise until black hits 3 lines
    4. place block, retract, return
    5. update counter
    6. next time count 2 black lines
    7. place block, retract, return
    8. update counter
    9. next time ocunt 1 black line
    10. place block, retract return
  */
  Position();
  ReadLineTracker();
  switch (StepIndex) {
    case 1:
      ArmBend.write(0);
      ArmBase.write(0);
      Wrist.write(70);//70-180, bent-straightout
      if (OrTheta < 200) {
        RgtMotorSpeed = 1600;
        LftMotorSpeed = 1400;
      }
      else {
        StepIndex = 2;
      }
      break;
    case 2:
      RgtMotorSpeed = 1600;
      LftMotorSpeed = 1400;
      if ((GripLightData < GripLightDark) && (!HitBlack)) {
        HitBlackCount++;
        HitBlack = true;
        if (HitBlackCount == HitBlackTarget) {
          StepIndex = 3;
        }
      }
      else if ((GripLightData > GripLightDark) && (HitBlack)) {
        HitBlack = false;
      }
      break;

    case 3:
      LftMotorSpeed = 1500;
      RgtMotorSpeed = 1500;
      ArmBend.write(0);
      ArmBase.write(0);
      Wrist.write(70);
      Grip.write(100);
      HitBlackCount = 0;
      HitBlackTarget--;
      break;
  }
  pickedUp++;
  if (pickedUp == 3) ModeIndex = 0;
  else Return();
  return;
}

//Mode 2

void Move() {//detected tesseract on wall, pick it up, turn, move under beam, then run DropOff
  //robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  Grip.write(100); // open grip
  ArmBend.write(165); // extend arm, grip above tesseract
  ArmBase.write(90);
  Wrist.write(170);
  delay(300);

  while (analogRead(GripLight) < 950) { // 950 --> light, over 1000 --> dark
    LftMtr.writeMicroseconds(1425);
  }
  LftMtr.writeMicroseconds(1500);

  ArmBase.write(175);//lower claw around tesseract
  ArmBend.write(175);
  Wrist.write(180);
  Grip.write(170); // close grip

  LftMtr.writeMicroseconds(1350);
  RgtMtr.writeMicroseconds(1350);
  delay(400);
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  RgtMtr.writeMicroseconds(1650);// turn right towards overhang
  LftMtr.writeMicroseconds(1350);
  delay(200);
  RgtMtr.writeMicroseconds(1500);
  LftMtr.writeMicroseconds(1500);

  for (int i = 1; i <= 19; i++) { //want 18 cm from wall
    Ping(UltrasonicPingSide);
    UltrasonicDistance = UltrasonicDistance / 10;
    AnyUse = 18 - UltrasonicDistance;
    RgtMtr.writeMicroseconds(1400 + AnyUse);
    LftMtr.writeMicroseconds(1400 - AnyUse);
    delay(25);
    AnyUse = -AnyUse;
    RgtMtr.writeMicroseconds(1400 + AnyUse);
    LftMtr.writeMicroseconds(1400 - AnyUse);
    delay(25);
  }
  for (int i = 1; i <= 50; i++) { //want 18 cm from wall
    Ping(UltrasonicPingSide);
    if ((!(UltrasonicDistance < 210)) || (UltrasonicDistance = 0)) UltrasonicDistance = 180;
    UltrasonicDistance = UltrasonicDistance / 10;
    AnyUse = UltrasonicDistance - 18;
    RgtMtr.writeMicroseconds(1600 + AnyUse);
    LftMtr.writeMicroseconds(1600 - AnyUse);
    delay(25);
    AnyUse = -AnyUse;
    RgtMtr.writeMicroseconds(1600 + AnyUse);
    LftMtr.writeMicroseconds(1600 - AnyUse);
    delay(25);
  }
  DropOff();
  return;
}

void DropOff() {//robot under/past overhang, reach up and attach tesseract, then compress and roll back, return to main switch check
  Grip.write(170);
  delay(500);
  ArmBase.write(80);
  delay(300);
  ArmBend.write(50);
  Wrist.write(18);
  delay(1000);
  while (analogRead(2) > 950) { // over 1000 --> light, less than 500 -->dark
    LftMtr.writeMicroseconds(1340);
    RgtMtr.writeMicroseconds(1370);
  }
  delay(200);
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  delay(500);
  ArmBase.write(85);
  ArmBend.write(40);
  Wrist.write(35);
  delay(500);
  Grip.write(100);
  for (int i = 85; i >= 55; i -= 5) {
    ArmBase.write(i);
    delay(100);
    ArmBend.write(115 - i);
  }
  for (int i = 80; i < 150; i += 5) {
    ArmBend.write(i);
    delay(100);
  }
  delay(1000);
  LftMtr.writeMicroseconds(1340);
  RgtMtr.writeMicroseconds(1370);
  delay(2000);
  LftMtr.writeMicroseconds (1500);
  RgtMtr.writeMicroseconds(1500);
  pickedUp++;
  return;
}
