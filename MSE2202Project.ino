#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>

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



//pins
const int LftMtrPin = 8;
const int RgtMtrPin = 0;//*******
const int ArmBasePin = 0;//********
const int ArmBendPin = 0;//********
const int WristPin = 0;//********
const int GripPin = 0;//********
const int HallRgt = A0;//********
const int HallLft = A0;//********
const int HallGrip = A0;//********
const int GripLight = A0;//********
const int UltraLft = 0;//********
const int ultraRgt = 0;//********

//SERANTESTCOMMENT


int MovFst = 2200;
int Stop = 1600;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);
  LftEncdr.zero();
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);
  RgtEncdr.zero();
  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  ArmBaseEncdr.zero();
  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBendEncdr.zero();
  pinMode(7, INPUT);
 
}

//functions
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



void loop() {



}
