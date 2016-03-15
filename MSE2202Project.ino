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

//Hall Sensor Stuff
#define NOFIELD 505L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  

//Line Tracker Stuff
unsigned LineTrackerData = 0;
//Data variables
unsigned long HallSensorValue = 0;
unsigned long UltrasonicDistance = 0;

Servo RghtMtr;
Servo LftMtr;
Servo ArmBend;
Servo ArmBase;
Servo Grip;
Servo Wrist;
I2CEncoder LftEncdr;
I2CEncoder RghtEncdr;


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
const int LftMtrPin = 5; //Correctly assigned 
const int RgtMtrPin = 4; //Do not change 
const int ArmBasePin = 0;//********
const int ArmBendPin = 0;//********
const int WristPin = 0;//********
const int GripPin = 0;//********
const int HallRgt = A0;//********
const int HallLft = A0;//********
const int HallGrip = A0;//********
const int GripLight = A0;//********
const int UltraLft = 0;//********
const int UltraRgt = 0;//********
const int UltrasonicPing = 0;
const int UltrasonicData = 0;
const int HallSensor1 = A0;
const int HallSensor2 = A1;
const int LineTracker = A2;

int lftspeed = 1500;
int rghtspeed = 1500;
long lftPosition;
long rghtPosition;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Set up two motors
  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);
  pinMode(RgtMtrPin, OUTPUT);
  RghtMtr.attach(RgtMtrPin);

  // Set up encoders DO NOT CHANGE ORDER
  RghtEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  RghtEncdr.setReversed(false);  // adjust for positive count when moving forward
  LftEncdr.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward
  
  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  
  
  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  
  pinMode(7, INPUT);
  
  pinMode(LineTracker, INPUT);
  
  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicData, INPUT);
 
}
void loop(){
  DebuggerModule();

  int timer = millis();
  //Serial.println(timer);
  Serial.print("Encoders L: ");
    Serial.print(LftEncdr.getRawPosition());
    Serial.print(", R: ");
    Serial.println(RghtEncdr.getRawPosition());
  
  if (timer < 1000){
    lftspeed = 1800;
    rghtspeed = 1800;
    //Serial.println("move");
  } else {
    lftspeed = 1500;
    rghtspeed = 1500;
  }
  //Serial.print(lftspeed);
  
  LftMtr.writeMicroseconds(lftspeed);
  RghtMtr.writeMicroseconds(rghtspeed);

  
}
//functions

void DebuggerModule(){
  //Debugger module -> all debugger code can go here
  #ifdef DEBUG_HALL_SENSOR
    Serial.println((analogRead(HallSensor1) - NOFIELD) * TOMILLIGAUSS / 1000);
    Serial.println((analogRead(HallSensor2) - NOFIELD) * TOMILLIGAUSS / 1000);
  #endif
  
  #ifdef DEBUG_ULTRASONIC
    Serial.print("Time (microseconds): ");
    Serial.print(UltrasonicDistance*58, DEC);
    Serial.print(", cm's: ");
    Serial.println(UltrasonicDistance);
  #endif
  
  #ifdef DEBUG_LINE_TRACKER
    Serial.print("Light Level: ");
    Serial.println(LineTrackerData, DEC);
  #endif
  
  #ifdef DEBUG_ENCODERS
  lftPosition = LftEncdr.getRawPosition();
  rghtPosition = RghtEncdr.getRawPosition();

  Serial.print("Encoders L: ");
  Serial.print(lftPosition);
  Serial.print(", R: ");
  Serial.println(rghtPosition);
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
  LineTrackerData = analogRead(LineTracker);
}
//Mode 1
void Look() {
  //if already found tesseract-> run 'Return', else-> robot starts looking for tesseracts, 
  //if detects tesseract stops and runs 'PickUp'
  //needs to keep track of position? for 'GoHome' /OR/ 'GoHome' can find home position from where it is
  //needs collision avoidance system -> runs 'Countermeasures'?
}
void Countermeasures(){
  //robot reacts to interference by other robot, after safe returns to 'Look'
}
void PickUp() {
  //robot has deteced tesseract in 'Look' and uses arm to pick it up, after picked up runs 'GoHome'
}
void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
};
void Return() {
  //robot is at start and has already picked up a tesseract, return to last position where tesseract was picked up, continue with 'Look'
}


//Mode 2
void Check(){
  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
}
void Move(){
//robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'
}



//requires timer system and tesseracts picked up counter 


