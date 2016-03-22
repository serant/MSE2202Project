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
#define DEBUG_ENCODERS

//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool StartTracking = false;
bool TurnRight = true;

//Hall Sensor Stuff
#define NOFIELD 505L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  
const unsigned HallThreshold = 20; //<- NEEDS TO BE MEASURED

//Mechanical Information
unsigned WheelPerimeter = 63; //perimeter of wheel in mm <- NEEDS TO BE MEASURED
unsigned ForwardSpeed = 1800; //speed of robot while looking in mode 1
unsigned LftSpeed = 1600;
unsigned RgtSpeed = 1600;
//Line Tracker Stuff
unsigned GripLightData = 0;

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
const int LftMtrPin = 5;//Correctly Assigned
const int RgtMtrPin = 4;//Do not change
const int ArmBasePin = 6;//********
const int ArmBendPin = 7;//********
const int WristPin = 0;//********
const int GripPin = 0;//********
const int HallRgt = A1;//********
const int HallLft = A0;//********
const int HallGrip = A5;//********
const int GripLight = A2;//********
const int UltrasonicPing = 2;
const int UltrasonicData = 3;
int MovFst = 2200;
int Stop = 1600;

// variables
unsigned int MotorSpeed;
unsigned int LftMotorSpeed;
unsigned int RgtMotorSpeed;
unsigned int LftMotorPos;
unsigned int RgtMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;


// Tracking Variables
long SvdLftPosition;
long SvdRgtPosition;
const double CE = 637;//pulses per revolution 
const double CF= ((3.14159*69.85)/CE); //Conversion factor, traslates encoder pulses to linear displacement
double DstnceLft = 0;
double DstnceRgt = 0; 
double Dstnce = 0;
double Theta = 0;
double SvdTheta = 0;
double XPstn = 0;
double YPstn = 0;
 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Set up two motors
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  // Set up encoders DO NOT CHANGE ORDER
  LftEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward
  RgtEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward
  //LftEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  //LftEncdr.setReversed(true);  // adjust for positive count when moving forward
  
  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  ArmBaseEncdr.zero();
  
  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBendEncdr.zero();
  pinMode(7, INPUT);
  
  pinMode(GripLight, INPUT);
  
  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicData, INPUT);

}
void loop(){
  DebuggerModule();

  int timer = millis();
  Position();

  if(timer > 8000){
    GoHome();
  }
  
  //Serial.println(timer);
  
  Serial.print("Encoders L: ");
  Serial.print(LftEncdr.getRawPosition());
  Serial.print(", R: ");
  Serial.println(RgtEncdr.getRawPosition());
  /*
  if (timer < 5000){
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
  */
  /*Look();
  
  if(StartTracking){
    trackPosition();
  }*/
}
//functions

void DebuggerModule(){
  //Debugger module -> all debugger code can go here
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

  LftMotorPos = LftEncdr.getRawPosition();
  RgtMotorPos = RgtEncdr.getRawPosition();

  Serial.print("Encoders L: ");
  Serial.print(LftMotorPos);
  Serial.print(", R: ");
  Serial.println(RgtMotorPos);

  #endif
}

void Ping(){
  //Ping Ultrasonic
  digitalWrite(UltrasonicPing, HIGH);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  digitalWrite(UltrasonicPing, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(UltrasonicData, HIGH, 10000)/58);
}

void readLineTracker(){
  GripLightData = analogRead(GripLight);
}
//Mode 1
void trackPosition(){
  if(EnableIncrement == false && LftMtr.read() <= 270){
    EnableIncrement = true;
  }  
  else if(EnableIncrement == false && LftMtr.read() > 270){
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
  LftEncdr.zero();
  RgtEncdr.zero();
  //Step 1 -> turn left
  if(StartLooking){
    RgtMtr.write(RgtMtr.read() + WheelPerimeter);
    LftMtr.write(LftMtr.read() - WheelPerimeter);
    StartLooking = false;
    StartTracking = true;
  }
  
  if(XPos < (CourseWidth - 600)){
    if((((analogRead(HallLft) - NOFIELD) * TOMILLIGAUSS/1000) < HallThreshold) || ((analogRead(HallRgt) - NOFIELD) * TOMILLIGAUSS/1000) < HallThreshold){
      RgtMtr.writeMicroseconds(ForwardSpeed);
      LftMtr.writeMicroseconds(ForwardSpeed);
    }
    else
      PickUp();
  }
  
  else{  //if reaches end of course width, turn right then left
    if(TurnRight){
      LftMtr.write(RgtMtr.read() + (3 * WheelPerimeter));
      TurnRight = false;
    }
    else if(!TurnRight){
      RgtMtr.write(LftMtr.read() + (3 * WheelPerimeter));
      TurnRight = true;
    }
  }
  
}
void Countermeasures(){
  //robot reacts to interference by other robot, after safe returns to 'Look'
}
void PickUp() {
  //robot has deteced tesseract in 'Look' and uses arm to pick it up, after picked up runs 'GoHome'
}
void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  SvdLftPosition = LftEncdr.getRawPosition();
  SvdRgtPosition = RgtEncdr.getRawPosition();
  Position();
  SvdTheta = (atan(XPstn/YPstn)*180)/3.14;
  Serial.println(SvdTheta);
  while (!(Theta < (SvdTheta +10) && Theta >(SvdTheta - 10))){
    Serial.println("Alinging Bot...");
    LftMtr.write(1500);
    RgtMtr.write(1300);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  int SvdLft = LftEncdr.getRawPosition();
  while (LftEncdr.getRawPosition() < (SvdLft + ((sqrt((XPstn*XPstn) + (YPstn*YPstn)))/CF)+10)){
     Serial.println("Moving towards origin...");
     LftMtr.write(2000);
     RgtMtr.write(2000);
  }
  
};
void Return() {
  //robot is at start and has already picked up a tesseract, return to last position where tesseract was picked up, continue with 'Look'
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  Position();
  SvdTheta = SvdTheta + 180;
  Serial.println(SvdTheta);
  while (!(Theta < (SvdTheta +10) && Theta >(SvdTheta - 10))){
    Serial.println("Alinging Bot...");
    LftMtr.write(1500);
    RgtMtr.write(1300);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  int SvdLft = LftEncdr.getRawPosition();
  while (LftEncdr.getRawPosition() < (SvdLft + ((sqrt((XPstn*XPstn) + (YPstn*YPstn)))/CF)+10)){
     Serial.println("Moving towards origin...");
     LftMtr.write(2000);
     RgtMtr.write(2000);
  }
}
void Position(){
  //Serial.println(CF,DEC);
  //Serial.println(RgtEncdr.getRawPosition());
  DstnceRgt = (CF * (RgtEncdr.getRawPosition())); // Distance traveled by right Wheel 
  //Serial.println(DstnceRgt);
  DstnceLft = CF * (LftEncdr.getRawPosition()); // Distnace traveled by left wheel 
  //Serial.println(DstnceLft);
  
  Dstnce = (DstnceRgt + DstnceLft)/2;
  Serial.print("Distnace: ");
  Serial.println(Dstnce);
  
  Theta = ((DstnceLft - DstnceRgt)/109) *(180/3.14); // Change in orientation, taking starting postion as Theta = 0
  if (Theta > 360){
    Theta = Theta - 360;
  } else if ( Theta < -360){
    Theta = Theta + 360;
  }
  Serial.print("Theta: ");
  Serial.println(Theta);
  
  XPstn = Dstnce * cos(Theta);
  YPstn = Dstnce * sin(Theta);
  Serial.print("X: ");
  Serial.print(XPstn);
  Serial.print( "Y: ");
  Serial.println(YPstn);
}


//Mode 2
void Check(){
  
  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
// Robo --> back and forth scanning motion
LftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
RgtMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
int lastHallReading = analogRead(HallGrip);
int LftEncoderCounter = LftEncdr.getRawPosition();
int RgtEncoderCounter = RgtEncdr.getRawPosition();
 
LftMotorSpeed = 1650;
LftMtr.writeMicroseconds(LftMotorSpeed);
for(LftEncoderCounter; LftEncoderCounter < 50; LftEncoderCounter++){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Left Encoder Forward: ");
  Serial.println(LftEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
LftMotorSpeed = 1350;
LftMtr.writeMicroseconds(LftMotorSpeed);
for(LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Left Encoder Backward: ");
  Serial.println(LftEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
LftMotorSpeed = 1500;
LftMtr.writeMicroseconds(LftMotorSpeed);
delay(200);


RgtMotorSpeed = 1650;
RgtMtr.writeMicroseconds(RgtMotorSpeed);
for(RgtEncoderCounter; RgtEncoderCounter < 50; RgtEncoderCounter++){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Right Encoder Forward: ");
  Serial.println(RgtEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
RgtMotorSpeed = 1350;
RgtMtr.writeMicroseconds(RgtMotorSpeed);
for(RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--){
  int currentHallReading = analogRead(HallGrip);
  Serial.print("Right Encoder Backward: ");
  Serial.println(RgtEncoderCounter);
  if(currentHallReading - lastHallReading > 20){
    return;
  }
}
RgtMotorSpeed = 1500;
RgtMtr.writeMicroseconds(RgtMotorSpeed);
delay(200);
}
void Move(){
//robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'
}



//requires timer system and tesseracts picked up counter 


