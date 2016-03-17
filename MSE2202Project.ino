#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>

const unsigned long CourseWidth = 6000; //course width in mm
unsigned long XPos = 0;

//DEBUGGERS -> uncomment to debug
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS

//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool StartTracking = false;
bool TurnRight = true;

//Hall Sensor Stuff
#define NOFIELD 505L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  
const unsigned HallThreshold = 20; //<- NEEDS TO BE MEASURED
unsigned int HallIdle;

//Mechanical Information
unsigned WheelPerimeter = 63; //perimeter of wheel in mm <- NEEDS TO BE MEASURED
unsigned ForwardSpeed = 1800; //speed of robot while looking in mode 1
unsigned LftSpeed = 1600;
unsigned RgtSpeed = 1600;
//Line Tracker Stuff
<<<<<<< HEAD
unsigned LineTrackerData = 0;
=======
unsigned GripLightData = 0;
>>>>>>> origin/master

//Data variables
unsigned long HallSensorValue = 0;
unsigned long UltrasonicDistance = 0;

Servo LftMtr;
Servo ArmBend;
Servo ArmBase;
Servo RgtMtr;
Servo Grip;
Servo Wrist;
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

//pins
<<<<<<< HEAD
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 6;
const int ArmBendPin = 7;
const int WristPin = 0;//********
const int GripPin = 0;//********
const int HallRgt = A1;
const int HallLft = A0;
const int HallGrip = A0;//************
const int GripLight = A2;
const int UltrasonicPing = 2;
const int UltrasonicData = 3;

int MovFst = 2200;
int Stop = 1600;

// variables
unsigned int MotorSpeed;
unsigned int LeftMotorSpeed;
unsigned int RightMotorSpeed;
unsigned int LefftMotorPos;
unsigned int RightMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;


// Tracking Variables
long SvdLftPosition;
long SvdRgtPosition;
const int CE = 637;//pulses per revolution 
const int CF= (3.14159*69.85)/CE; //Conversion factor, traslates encoder pulses to linear displacement
int DstnceLft = 0;
int DstnceRgt = 0; 
int Dstnce = 0;
int Theta = 0;
int SvdTheta = 0;
int XPstn = 0;
int YPstn = 0;
 

void setup() {
  Serial.begin(9600);
  Wire.begin();

=======
  
  // Set up two motors
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  // Set up encoders DO NOT CHANGE ORDER
  RgtEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward
  
  LftEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward
  
>>>>>>> origin/master
  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin); // 37 folded, 180 out

  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin); // 180 folded, 0 out
  pinMode(7, INPUT);
<<<<<<< HEAD

  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicData, INPUT);

<<<<<<< HEAD
  HallIdle = (analogRead(HallLft) + analogRead(HallRgt) / 2); ///*********works???

=======
}
void loop() {
  DebuggerModule();

  int timer = millis();
  Position();

  //if(timer > 8000){
    //GoHome();
  //}
  
  //Serial.println(timer);
  Serial.print("Encoders L: ");
  Serial.print(LftEncdr.getRawPosition());
  Serial.print(", R: ");
  Serial.println(RgtEncdr.getRawPosition());
  
  if (timer < 1000){
    LftSpeed = 1800;
    RgtSpeed = 1800;
    //Serial.println("move");
  } else {
    LftSpeed = 1500;
    RgtSpeed = 1500;
  }
  //Serial.print(lftspeed);
  
  LftMtr.writeMicroseconds(LftSpeed);
  RgtMtr.writeMicroseconds(RgtSpeed);
  
  Look();
<<<<<<< HEAD
  if (StartTracking) {
=======
    trackPosition();
  }
}
//functions

void DebuggerModule() {
  //Debugger module -> all debugger code can go here

=======
  #ifdef DEBUG_HALL_SENSOR
    Serial.println((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000);
    Serial.println((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000);
  #endif
  
  #ifdef DEBUG_ULTRASONIC
    Serial.print("Time (microseconds): ");
    Serial.print(UltrasonicDistance*58, DEC);
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
>>>>>>> origin/master
}

void Ping() {
  //Ping Ultrasonic
  digitalWrite(UltrasonicPing, HIGH);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  digitalWrite(UltrasonicPing, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(UltrasonicData, HIGH, 10000) / 58);
}

=======
void readLineTracker(){
  GripLightData = analogRead(GripLight);
}
//Mode 1
void trackPosition(){
  if(EnableIncrement == false && LftMtr.read() <= 270){
>>>>>>> origin/master
    EnableIncrement = true;
  }
  else if (EnableIncrement == false && LftMtr.read() > 270) {
    XPos += WheelPerimeter;
    EnableIncrement = false;
  }
}
//Mode 1
void Look() {
  //if already found tesseract-> run 'Return', else-> robot starts looking for tesseracts,
  //if detects tesseract stops and runs 'PickUp'
  //needs to keep track of position? for 'GoHome' /OR/ 'GoHome' can find home position from where it is
  //needs collision avoidance system -> runs 'Countermeasures'?

  //Step 1 -> turn left
  if (StartLooking) {
    RgtMtr.write(RgtMtr.read() + WheelPerimeter);
    LftMtr.write(LftMtr.read() - WheelPerimeter);
    StartLooking = false;
    StartTracking = true;
  }
<<<<<<< HEAD

  if (XPos < (CourseWidth - 600)) {
    if ((((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) || ((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS / 1000) < HallThreshold) {
=======
  
  if(XPos < (CourseWidth - 600)){
    if((((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS/1000) < HallThreshold) || ((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS/1000) < HallThreshold){
>>>>>>> origin/master
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
void Countermeasures() {
  //robot reacts to interference by other robot, after safe returns to 'Look'
}

unsigned HallLftRead, HallRgtRead;
int turn;
void PickUp() {
  //robot has deteced tesseract in 'Look' and uses arm to pick it up, after picked up runs 'GoHome'

  //********something to determine position and save it*********

  HallLftRead = analogRead(HallLft);
  HallRgtRead = analogRead(HallRgt);
  if ((HallLftRead - HallIdle > 5) || (HallLftRead - HallIdle < -5)) {
    turn = 1;//tess to left
  }
  if ((HallRgtRead - HallIdle > 5) || (HallRgtRead - HallIdle < -5)) {
    if (turn == 1) turn = 2; // tess in middle
    else turn = 3;  // tess to right
  }
  switch (turn) {
    case 1: {
        RgtMtr.write(1450); ///this should align robot a bit to left  *******test #s
        LftMtr.write(1400);
        delay(500);
        LftMtr.write(1800);
        RgtMtr.write(1800);
        delay(500);
        LftMtr.write(1600);
        RgtMtr.write(1600);
      }
    case 2: {
        LftMtr.write(1450); ///should align robot bit to right **********test #s
        RgtMtr.write(1400);
        delay(500);
        LftMtr.write(1800);
        RgtMtr.write(1800);
        delay(500);
        LftMtr.write(1600);
        RgtMtr.write(1600);
      }
    case 3: {
        while (UltrasonicDistance > 5 || UltrasonicDistance < 3) {  ///align tesseract in middle *******test #s, in cm
          ////******want to use IR or some other form of distance? think it may work better, especially for small distance
          Ping();
          LftMtr.write(1620);
          RgtMtr.write(1580);
        }

        Grip.write(/*open*/100);  /////pick up tesseract *********test #s
        Wrist.write(/*angled*/100);  //******* test #s
        ArmBase.write(50);      // 37 folded, 180 out
        ArmBend.write(150);    //180 folded, 0 out
        Grip.write(/*closed*/0);   ///*********  test #s
        ArmBase.write(50);
        ArmBend.write(160);
        if(ArmBase.read() == 100) return;    ///////requires ArmBase not be at 100 when not holding something (folded up empty ~37)
      }
  }
}

void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  SvdLftPosition = LftEncdr.getRawPosition();
  SvdRgtPosition = RgtEncdr.getRawPosition();
  Position();
  SvdTheta = atan(XPstn/YPstn);
  while (Theta > SvdTheta + (3.14/16) && Theta <SvdTheta - (3.14/16)){
    LftMtr.write(1600);
    RgtMtr.write(1400);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  int SvdLft = LftEncdr.getRawPosition();
  while (LftEncdr.getRawPosition() < SvdLft + ((sqrt((XPstn*XPstn) + (YPstn*YPstn)))/CF)){
     LftMtr.write(1600);
     RgtMtr.write(1600);
  }
  
};
void Return() {
  //robot is at start and has already picked up a tesseract, return to last position where tesseract was picked up, continue with 'Look'
}
void Position(){
  DstnceRgt = CF * (RgtEncdr.getRawPosition()); // Distance traveled by left Wheel 
  Serial.println(DstnceRgt);
  DstnceLft = CF * (LftEncdr.getRawPosition()); // Distnace traveled by right wheel 
  Serial.println(DstnceLft);
  
  Dstnce = (DstnceRgt + DstnceLft)/2;
  Serial.println(Dstnce);
  
  Theta = (DstnceLft - DstnceRgt)/175; // Change in orientation, taking starting postion as Theta = 0
  Serial.println(Theta);
  
  XPstn = Dstnce * cos(Theta);
  YPstn = Dstnce * sin(Theta);
  Serial.println(XPstn);
  Serial.println(YPstn);
}


//Mode 2
<<<<<<< HEAD
void Check() {
=======
  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
// Robo --> back and forth scanning motion
LeftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
RightMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
int lastHallReading = analogRead(HallGrip);
int LftEncoderCounter = LftEncdr.getRawPosition();
int RgtEncoderCounter = RgtEncdr.getRawPosition();
 
LeftMotorSpeed = 1650;
LftMtr.writeMicroseconds(LeftMotorSpeed);
for(LftEncoderCounter; LftEncoderCounter < 50; LftEncoderCounter++){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Left Encoder Forward: ");
  Serial.println(LftEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
LeftMotorSpeed = 1350;
LftMtr.writeMicroseconds(LeftMotorSpeed);
for(LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Left Encoder Backward: ");
  Serial.println(LftEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
LeftMotorSpeed = 1500;
LftMtr.writeMicroseconds(LeftMotorSpeed);
delay(200);


RightMotorSpeed = 1650;
RgtMtr.writeMicroseconds(RightMotorSpeed);
for(RgtEncoderCounter; RgtEncoderCounter < 50; RgtEncoderCounter++){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Right Encoder Forward: ");
  Serial.println(RgtEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
RightMotorSpeed = 1350;
RgtMtr.writeMicroseconds(RightMotorSpeed);
for(RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Right Encoder Backward: ");
  Serial.println(RgtEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
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


