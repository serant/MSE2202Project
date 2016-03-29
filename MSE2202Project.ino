#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>

unsigned long CourseWidth = 6000; //course width in mm
unsigned long XPos = 0;

//DEBUGGERS -> uncomment to debug
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS

//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool TurnRight = true;

//Hall Sensor Stuff
#define NOFIELD 505L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V
const unsigned HallThreshold = 5;

//Mechanical Information
unsigned WheelPerimeter = 63; //perimeter of wheel in mm <- NEEDS TO BE MEASURED
unsigned ForwardSpeed = 1800; //speed of robot while looking in mode 1
//Line Tracker Stuff
unsigned LineTrackerData = 0;
unsigned GripLightData = 0;

//motion
const unsigned Stop = 1600;
//deviation above = forward, below = backward

//Data variables
unsigned long HallSensorValue = 0;
unsigned long UltrasonicDistance = 0;

Servo LftMtr;
Servo ArmBend;  // out->folded 0 -180
Servo ArmBase;  // folded ->out 37-179
Servo RgtMtr;
Servo Grip;     //150-155 closed grip, 90 open
Servo Wrist;    //70 min folded, 180 staight out
I2CEncoder LftEncdr;
I2CEncoder RgtEncdr;

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

// variables
unsigned int MotorSpeed;
unsigned int LeftMotorSpeed;
unsigned int RightMotorSpeed;
unsigned int LeftMotorPos;
unsigned int RightMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;


// Tracking Variables
long SvdLftPosition;
long SvdRgtPosition;
const int CE = 637;//pulses per revolution
const int CF = (3.14159 * 69.85) / CE; //Conversion factor, traslates encoder pulses to linear displacement
int DstnceLft = 0;
int DstnceRgt = 0;
int Dstnce = 0;
int Theta = 0;
int SvdTheta = 0;
int XPstn = 0;
int YPstn = 0;

//pins /
//remaining A1
//12, 13 used for switches to select mode
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 6;
const int ArmBendPin = 7;
const int WristPin = 11;
const int GripPin = 10;
const int HallRgt = A5;
const int HallLft = A4;
const int HallGrip = A3;
const int GripLight = A2;
const int UltrasonicPing = 2;//data return in 3
const int UltrasonicPingSide = 8;//data return in 9

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Set up two motors
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  pinMode(GripPin, OUTPUT);
  Grip.attach(GripPin);

  pinMode(WristPin, OUTPUT);
  Wrist.attach(WristPin);

  // Set up encoders DO NOT CHANGE ORDER
  RgtEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward

  LftEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward

  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin); // 37 folded, 180 out

  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin); // 180 folded, 0 out
  pinMode(7, INPUT);

  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicPing + 1, INPUT);
  pinMode (UltrasonicPingSide, OUTPUT);
  pinMode(UltrasonicPingSide + 1, INPUT);

  Ping(UltrasonicPing);
  CourseWidth = UltrasonicDistance * 10;
  Serial.print ("coursewidth   ");
  Serial.println(CourseWidth);

  if (digitalRead(13)) {
    if (digitalRead(12)) {
      ModeIndex = 2; // switch 3 and 1 on (down)
    }
    else ModeIndex = 1; // switch 3 off (up), 1 on (down)
  }
  else ModeIndex = 0; // switch 1 off(up)

  Serial.println("Setup Done");
}

void loop() {
  //***************************stuff running through every time
  DebuggerModule();
  int timer = millis() / 1000;

  //if(timer > 8000){
  //GoHome();
  //}

  if (timer % 1000 < 50) {
    Serial.print("mode  ");
    Serial.println(ModeIndex);
  }
  switch (ModeIndex) {
    case 0: //******************************sitting around waiting, use this mode to test stuff, then clear

      Serial.print("lftpstn =  ");
      Serial.println(LftEncdr.getRawPosition());
      Serial.print("rgtpstn =  ");
      Serial.println(RgtEncdr.getRawPosition());
      XPos = (LftEncdr.getRawPosition() + RgtEncdr.getRawPosition()) * CF / 2;
      Serial.print("xpos =  ");
      Serial.println(XPos);
      Serial.print("hall left  ");
      Serial.println((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000);
      Serial.print("hal right  ");
      Serial.println((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000);

      if (XPos < (CourseWidth - 100)) {

        if ((analogRead(HallLft) - NOFIELD > HallThreshold) || (analogRead(HallLft) - NOFIELD < -HallThreshold) || (analogRead(HallRgt) - NOFIELD > HallThreshold) || (analogRead(HallRgt) - NOFIELD < -HallThreshold)) {
          RgtMtr.writeMicroseconds(ForwardSpeed+300);
          LftMtr.writeMicroseconds(ForwardSpeed+300);
          Serial.println("going");
        }
        else
        Serial.println("detected tesseract");
        RgtMtr.writeMicroseconds(Stop);
        LftMtr.writeMicroseconds(Stop);
        delay(1000);
        //   PickUp();//******runs pickup function***************************************uncomment
      }
      else { //if reaches end of course width, turn right then left
        if (TurnRight) {
          LftMtr.write(RgtMtr.read() + (3 * WheelPerimeter));
        }
        else {
          RgtMtr.write(LftMtr.read() + (3 * WheelPerimeter));
        }
        TurnRight = !TurnRight;
      }
      LftMtr.write(1700);
      RgtMtr.write(1700);
      break;

    case 1: //**********************************mode 1 base
      Serial.println("went to 1");
      ModeIndex = 0;

      break;

    case 2:  //********************mode 2 base
      Serial.println("went to 2");
      ModeIndex = 0;
      break;

    case 3:

      break;

    case 4:

      break;

    case 5:

      break;
    case 6:

      break;
    case 7:

      break;
      //etc. add as needed
  }
}

//functions

void DebuggerModule() {
  //Debugger module -> all debugger code can go here

#ifdef DEBUG_HALL_SENSOR
  Serial.println((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000);
  Serial.println((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000);
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

  LftPosition = LftEncdr.getRawPosition();
  RgtPosition = RgtEncdr.getRawPosition();

  Serial.print("Encoders L: ");
  Serial.print(LftPosition);
  Serial.print(", R: ");
  Serial.println(RgtPosition);

#endif
}

void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) / 58);//returns distance in cm
}

void ReadLineTracker() {
  GripLightData = analogRead(GripLight);
}

void TrackPosition() {
  if (EnableIncrement == false && LftMtr.read() <= 270) {
    EnableIncrement = true;
  }
  else if (EnableIncrement == false && LftMtr.read() > 270) {
    XPos += WheelPerimeter;
    EnableIncrement = false;
  }
}

void Countermeasures() {//ignore?
  //robot reacts to interference by other robot, after safe returns to 'Look'
}

unsigned HallLftRead, HallRgtRead;
int turn;
void PickUp() {
  //robot has deteced tesseract and uses arm to pick it up, after picked up runs 'GoHome'

  if ((analogRead(HallLft) - NOFIELD > 5) || (analogRead(HallLft) - NOFIELD < -5)) {
    turn = 1;//tess to left
  }
  if ((analogRead(HallRgt) - NOFIELD > 5) || (analogRead(HallRgt) - NOFIELD < -5)) {
    if (turn == 1) turn = 3; // tess in middle
    else turn = 2;  // tess to right
  }
  switch (turn) {
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
        LftMtr.write(1600);
        RgtMtr.write(1400);
        break;
        UltrasonicDistance = 0;
        LftMtr.write(1500);
        RgtMtr.write(1500);

        //Grip.write(/*open*/100);  /////pick up tesseract *********test #s
        //Wrist.write(/*angled*/100);  //******* test #s
        ArmBase.write(110);      // 37 folded, 180 out
        ArmBend.write(150);    //180 folded, 0 out
        delay(500);
        //Grip.write(/*closed*/0);   ///*********  test #s
        delay(500);
        ArmBase.write(40);
        ArmBend.write(160);
      }
  }
}

void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  SvdLftPosition = LftEncdr.getRawPosition();
  SvdRgtPosition = RgtEncdr.getRawPosition();
  Position();
  SvdTheta = atan(XPstn / YPstn);
  while (Theta > SvdTheta + (3.14 / 16) && Theta < SvdTheta - (3.14 / 16)) {
    LftMtr.write(1600);
    RgtMtr.write(1400);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  int SvdLft = LftEncdr.getRawPosition();
  while (LftEncdr.getRawPosition() < SvdLft + ((sqrt((XPstn * XPstn) + (YPstn * YPstn))) / CF)) {
    LftMtr.write(1600);
    RgtMtr.write(1600);
  }

};
void Return() {
  //robot is at start and has already picked up a tesseract, return to last position where tesseract was picked up, continue with 'Look'
}
void Position() {
  DstnceRgt = CF * (RgtEncdr.getRawPosition()); // Distance traveled by left Wheel
  Serial.println(DstnceRgt);
  DstnceLft = CF * (LftEncdr.getRawPosition()); // Distnace traveled by right wheel
  Serial.println(DstnceLft);

  Dstnce = (DstnceRgt + DstnceLft) / 2;
  Serial.println(Dstnce);

  Theta = (DstnceLft - DstnceRgt) / 175; // Change in orientation, taking starting postion as Theta = 0
  Serial.println(Theta);

  XPstn = Dstnce * cos(Theta);
  YPstn = Dstnce * sin(Theta);
  Serial.println(XPstn);
  Serial.println(YPstn);
}


//Mode 2
void Check() {
  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
  // Robo --> back and forth scanning motion
  LeftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
  RightMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
  int lastHallReading = analogRead(HallGrip);
  LeftMotorPos = LftEncdr.getRawPosition();
  RightMotorPos = RgtEncdr.getRawPosition();

  LeftMotorSpeed = 1650;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  for (LeftMotorPos; LeftMotorPos < 50; LeftMotorPos++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Forward: ");
    Serial.println(LeftMotorPos);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LeftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  for (LeftMotorPos; LeftMotorPos > 0; LeftMotorPos--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Backward: ");
    Serial.println(LeftMotorPos);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LeftMotorSpeed = 1500;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  delay(200);

  RightMotorSpeed = 1650;
  RgtMtr.writeMicroseconds(RightMotorSpeed);
  for (RightMotorPos; RightMotorPos < 50; RightMotorPos++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Forward: ");
    Serial.println(RightMotorPos);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  RightMotorSpeed = 1350;
  RgtMtr.writeMicroseconds(RightMotorSpeed);
  for (RightMotorPos; RightMotorPos > 0; RightMotorPos--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Backward: ");
    Serial.println(RightMotorPos);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  RightMotorSpeed = 1500;
  RgtMtr.writeMicroseconds(RightMotorSpeed);
  delay(200);
}
void Move() {
  //robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'
}

//requires timer system and tesseracts picked up counter
