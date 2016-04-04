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
bool start = true;

//Hall Sensor Stuff
#define NOFIELDGRIP 505L
#define NOFIELDRGT 512L
#define NOFIELDLFT 503L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  
const unsigned HallThreshold = 6; //difference at which detects magnet
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
Servo ArmBend;    //out->folded 0 ->180
Servo ArmBase;    //folded-> out 40->180
Servo RgtMtr;
Servo Grip;       //100 open (grip hits acrylic), 170 closed max, 150 parallel
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
unsigned long CourseWidth = 150; //course width in cm, has to be set prior to running
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
bool Black = false;
int BlockNumber = 1;
int Line = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  pinMode(GripPin, OUTPUT);
  Grip.attach(GripPin);
  Grip.write(160);

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
  ArmBase.write(100);

  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBend.write(115);

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
  /*
    // Allign with wall
    while (!((OrTheta > -275) && (OrTheta < -265))) {
    LftMtr.write(1700);
    RgtMtr.write(1300);
    Serial.println(OrTheta);
    Position();
    }

    // Move towards wall
    Ping(2);
    while (UltrasonicDistance < 21 && UltrasonicDistance != 0) {
    LftMtr.write(1300);
    RgtMtr.write(1300);
    Ping(2);
    }

    // Turn towards orientation Theta
    while (!(OrTheta < 5 && OrTheta > -5 )) {
    LftMtr.write(1700);
    RgtMtr.write(1300);
    Position();
    }
  */
  // Set up arm
  ArmBend.write(115);
  ArmBase.write(100);
  Wrist.write(80);
  Grip.write(160);
  Line = 0;
  delay(1000);

  while (true) {
    Serial.println(analogRead(GripLight));
    // Serial.print("Line: ");
    // Serial.println(Line);
    // Serial.print("Block: ");
    // Serial.println(BlockNumber);
    Ping(2);
    Serial.println(UltrasonicDistance);
    while (UltrasonicDistance < 21 && UltrasonicDistance != 0) {
      LftMtr.write (1350);
      RgtMtr.write (1350);
      Ping(2);
    }
    while (UltrasonicDistance > 23 && UltrasonicDistance != 0) {
      LftMtr.write (1650);
      RgtMtr.write (1650);
      Ping(2);
    }
    if ((analogRead(GripLight) <= 920) || (analogRead(GripLight) >= 990)) { // Light
      Serial.println("Turning...");
      LftMtr.write (1650);
      RgtMtr.write (1350);
      delay(50);
      LftMtr.write (1500);
      RgtMtr.write (1500);
      delay(50);
      Black = false;
    } else if ((920 < analogRead(GripLight)) && (analogRead(GripLight) < 990) && Black == false) { // Black line
      Line++;
      Black = true;
      if ((Line == 3 && BlockNumber == 1) || (Line == 2 && BlockNumber == 2) || (Line == 1 && BlockNumber == 3)) {
        BlockNumber++;
        break;
      } else {
        Black = false;
        LftMtr.write (1650);
        RgtMtr.write (1350);
        delay(50);
        LftMtr.write (1500);
        RgtMtr.write (1500);
        delay(50);
      }
    }
  }
  LftMtr.write (1500);
  RgtMtr.write (1500);
  Serial.println("Opening Claw...");
  Grip.write(105);
  delay(500);
  ArmBase.write(80);
  delay(500);
  // Return Function
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
}

void ReadLineTracker() {
  GripLightData = analogRead(GripLight);
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
}

/*void Look() {
  //if already found tesseract-> run 'Return', else-> robot starts looking for tesseracts,
  //if detects tesseract stops and runs 'PickUp'
  //needs to keep track of position? for 'GoHome' /OR/ 'GoHome' can find home position from where it is

  LftEncdr.zero();
  RgtEncdr.zero();

  //Step 1 -> turn left
  if (StartLooking) {
    RgtMtr.write(RgtMtr.read() + WheelPerimeter);
    LftMtr.write(LftMtr.read() - WheelPerimeter);
    StartLooking = false;
    StartTracking = true;
  }

  if (XPos < (CourseWidth - 600)) {
    if ((((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) || ((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) {

      if (XPos < (CourseWidth - 600)) {
        if ((((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) || ((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) {
          RgtMtr.writeMicroseconds(ForwardSpeed);
          LftMtr.writeMicroseconds(ForwardSpeed);
        }
        else
          PickUp();
      }

      else { //if reaches end of course width, turn right then left
        if (TurnRight) {
          LftMtr.write(RgtMtr.read() + (3 * WheelPerimeter));
          TurnRight = false;
        }
        else if (!TurnRight) {
          RgtMtr.write(LftMtr.read() + (3 * WheelPerimeter));
          TurnRight = true;
        }
      }
    }
  }
  }*/ ////this function is now run in main loop, keeping only for reference to before changes

void PickUp() {
  //robot has deteced tesseract and uses arm to pick it up, after picked up runs 'GoHome'
  while (!((analogRead(HallGrip) - NOFIELDGRIP > HallThreshold) || (analogRead(HallGrip) - NOFIELDGRIP < -HallThreshold))) {
    //drop tesseract if not magnetic, first run will drop anyway
    ArmBase.write(45);
    ArmBend.write(0);
    delay(500);
    Grip.write(90);
    delay(200);
    ArmBend.write(180);

    if ((analogRead(HallLft) - NOFIELDLFT > HallThreshold) || (analogRead(HallLft) - NOFIELDLFT < -HallThreshold)) {
      AnyUse = 1;//tess to left
    }
    if ((analogRead(HallRgt) - NOFIELDRGT > HallThreshold) || (analogRead(HallRgt) - NOFIELDRGT < -HallThreshold)) {
      if (AnyUse == 1) AnyUse = 3; // tess in middle
      else AnyUse = 2;  // tess to right
    }
    switch (AnyUse) {
      case 1:
        RgtMtr.write(1450); ///align robot a bit to left  *******test #s
        LftMtr.write(1400);
        delay(500);
        LftMtr.write(1800);
        RgtMtr.write(1800);
        delay(500);
        LftMtr.write(1600);
        RgtMtr.write(1600);
        break;
      case 2:
        LftMtr.write(1450); ///align robot bit to right **********test #s
        RgtMtr.write(1400);
        delay(500);
        LftMtr.write(1800);
        RgtMtr.write(1800);
        delay(500);
        LftMtr.write(1600);
        RgtMtr.write(1600);
        break;
      case 3:
        while (UltrasonicDistance != 5) { ///align tesseract in middle *******test #s, in cm
          Ping(UltrasonicPingSide);
          if (UltrasonicDistance < 5) AnyUse = 100;
          else AnyUse = -100;
          LftMtr.write(Stop + AnyUse);
          RgtMtr.write(Stop - AnyUse);
          break;

          UltrasonicDistance = 0;
          LftMtr.write(1600);
          RgtMtr.write(1600);

          Grip.write(90);  //open grip
          Wrist.write(100);
          ArmBase.write(110);      // 37 folded, 180 out
          ArmBend.write(150);    //180 folded, 0 out
          delay(500);
          Grip.write(150);   //close grip
          delay(500);
          ArmBase.write(40);
          ArmBend.write(160);
          Wrist.write(100);
        }
    }
  }
  return;//good tesseract
}

void Position() {
  // PickUpTheta, FindTheta, SvdRgtEncdr, SvdLftEncdr

  // Distance travelled
  DelRgt = (CF * ((RgtEncdr.getRawPosition()))); // Instantaneous Distance traveled by right Wheel
  DelLft = CF * ((LftEncdr.getRawPosition())); // Instantaneous Distnace traveled by left wheel
  DelDsp = (DelRgt + DelLft) / 2; //Instantaneous Distance traveled by the centerpoint of the robot
  Dsp = Dsp + DelDsp; //Current Displacement
  // Serial.print("Displacement: ");
  // Serial.println(Dsp);

  OrTheta = ((DelRgt - DelLft) / 109) * (180 / PI); // Change in orientation, taking starting postion as Theta = 0
  //OrTheta = OrTheta + dTheta; //Orientation of robot
  OrTheta = (int)OrTheta % 360; //If the magnitude of the orientation is greater than 360

  // Serial.print("Orientation Theta: ");
  // Serial.println(OrTheta);

  XPstn = DelDsp * cos(OrTheta * PI / 180);
  YPstn = DelDsp * sin(OrTheta * PI / 180);
  //XPstn = XPstn + dXPstn;
  //YPstn = YPstn + dYPstn;
  // Serial.print("X: ");//X coordinates of the robot (right is positive)
  // Serial.print(XPstn);
  // Serial.print( "Y: ");//Y coordinates of the robot (up is positive)
  PolTheta = atan(YPstn / XPstn); //The polar angle of the position of the robot

  RawLftPrv = LftEncdr.getRawPosition();
  RawRgtPrv = RgtEncdr.getRawPosition();
  PrvOrTheta = OrTheta;
}

void GoHome(int done) {
  //robot calculates and saves position and returns to base after tesseract picked up,
  Position();
  for (int i = 0; i > 0; i++) {

    SvdDsp = Dsp;
  }
  while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta - 5))) {
    Serial.println("Alinging Bot...");
    LftMtr.write(1500);
    RgtMtr.write(1300);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);

  while (Dsp > 10) {
    Serial.println("Moving towards origin...");
    LftMtr.write(2000);
    RgtMtr.write(2000);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  if (done = 0) ModeIndex = 0;
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
}

void PlaceTesseract() {
  /*
    // Allign with wall
    while (!((OrTheta > -275) && (OrTheta < -265))) {
    LftMtr.write(1700);
    RgtMtr.write(1300);
    Serial.println(OrTheta);
    Position();
    }

    // Move towards wall
    Ping(2);
    while (UltrasonicDistance < 21 && UltrasonicDistance != 0) {
    LftMtr.write(1300);
    RgtMtr.write(1300);
    Ping(2);
    }

    // Turn towards orientation Theta
    while (!(OrTheta < 5 && OrTheta > -5 )) {
    LftMtr.write(1700);
    RgtMtr.write(1300);
    Position();
    }
  */
  // Set up arm
  ArmBend.write(115);
  ArmBase.write(100);
  Wrist.write(80);
  Grip.write(160);
  Line = 0;
  delay(1000);

  while (true) {
    Serial.println(analogRead(GripLight));
    // Serial.print("Line: ");
    // Serial.println(Line);
    // Serial.print("Block: ");
    // Serial.println(BlockNumber);
    Ping(2);
    Serial.println(UltrasonicDistance);
    while (UltrasonicDistance < 21 && UltrasonicDistance != 0) {
      LftMtr.write (1350);
      RgtMtr.write (1350);
      Ping(2);
    }
    while (UltrasonicDistance > 23 && UltrasonicDistance != 0) {
      LftMtr.write (1650);
      RgtMtr.write (1650);
      Ping(2);
    }
    if ((analogRead(GripLight) <= 920) || (analogRead(GripLight) >= 990)) { // Light
      Serial.println("Turning...");
      LftMtr.write (1650);
      RgtMtr.write (1350);
      delay(50);
      LftMtr.write (1500);
      RgtMtr.write (1500);
      delay(50);
      Black = false;
    } else if ((920 < analogRead(GripLight)) && (analogRead(GripLight) < 990) && Black == false) { // Black line
      Line++;
      Black = true;
      if ((Line == 3 && BlockNumber == 1) || (Line == 2 && BlockNumber == 2) || (Line == 1 && BlockNumber == 3)) {
        BlockNumber++;
        break;
      } else {
        Black = false;
        LftMtr.write (1650);
        RgtMtr.write (1350);
        delay(50);
        LftMtr.write (1500);
        RgtMtr.write (1500);
        delay(50);
      }
    }
  }
  LftMtr.write (1500);
  RgtMtr.write (1500);
  Serial.println("Opening Claw...");
  Grip.write(105);
  delay(500);
  ArmBase.write(80);
  delay(500);
  // Return Function
}

//Mode 2
/*
  void Check() {

  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'

  // Robo --> back and forth scanning motion
  LftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
  RgtMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
  int lastHallread = analogRead(HallGrip);
  int LftEncoderCounter = LftEncdr.getRawPosition();
  int RgtEncoderCounter = RgtEncdr.getRawPosition();

  ArmBase.write(100); // 37 - 179 folded to out
  ArmBend.write(110); // 0 -180 out to folded

  LftMotorSpeed = 1650;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter < 40; LftEncoderCounter++) {
    int currentHallread = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
    Serial.print("Left Encoder Forward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallread - lastHallread > 15) {
      return;
    }
  }
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallread = analogRead(HallGrip);
    Serial.print("Left Encoder Backward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallread - lastHallread > 15) {
      return;
    }
  }
  LftMotorSpeed = 1500;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  delay(200);

  RgtMotorSpeed = 1650;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter < 40; RgtEncoderCounter++) {
    int currentHallread = analogRead(HallGrip);
    Serial.print("Right Encoder Forward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallread - lastHallread > 15) {
      return;
    }
  }
  RgtMotorSpeed = 1350;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--) {
    int currentHallread = analogRead(HallGrip);
    Serial.print("Right Encoder Backward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallread - lastHallread > 15) {
      return;
    }
  }
  RgtMotorSpeed = 1500;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  delay(200);
  }*/
//put check in main switch statement,

void Move() {//detected tesseract on wall, pick it up, turn, move under beam, then run DropOff
  //robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  while (UltrasonicDistance > 21 || UltrasonicDistance < 10) {
    Ping(UltrasonicPing);
    LftMtr.writeMicroseconds(1650);
    RgtMtr.writeMicroseconds(1650);
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  Grip.write(90); // open grip
  ArmBend.write(165); // extend arm, grip above tesseract
  ArmBase.write(165);
  Wrist.write(170);
  delay(300);

  while (analogRead(GripLight) < 950) { // 950 --> light, over 1000 --> dark
    LftMtr.writeMicroseconds(1425);
  }
  LftMtr.writeMicroseconds(1500);

  ArmBase.write(175);//lower claw around tesseract
  ArmBend.write(175);
  Wrist.write(180);
  Grip.writeMicroseconds(150); // close grip

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
}

void DropOff() {//robot under/past overhang, reach up and attach tesseract, then compress and roll back, return to main switch check
  ArmBend.writeMicroseconds(20); // extend arm uwards
  ArmBase.writeMicroseconds(90);
  Wrist.write(70); // 70 --> bent, 180 --> straight
  delay(500);
  while (analogRead(2) > 950) { // over 1000 --> light, less than 500 -->dark
    LftMtr.writeMicroseconds(1350);
    RgtMtr.writeMicroseconds(1350);
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  ArmBend.writeMicroseconds(0);
  Wrist.write(70);
  Grip.writeMicroseconds(90); // open grip
  delay(100);
  ArmBend.writeMicroseconds(180); //fold up arm
  ArmBase.writeMicroseconds(37);

  for (int i = millis(); i - millis() < 2000; i = millis()) {
    LftMtr.writeMicroseconds(1400);
    RgtMtr.writeMicroseconds(1400);
  }
  LftMtr.writeMicroseconds (1500);
  RgtMtr.writeMicroseconds(1500);
  pickedUp++;
}
