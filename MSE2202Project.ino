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




int MovFst = 2200;
int Stop = 1600;
bool tf = true;

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
  digitalWrite(7, HIGH);

}

//matt comment
void loop() {
  if (!digitalRead(7)) {
    tf = !tf;
    delay(300);
  }
  if (tf) {
    LftMtr.writeMicroseconds(MovFst);
  }
  else {
    if (LftMtr.readMicroseconds() != Stop) {
      LftMtr.writeMicroseconds(Stop);
    }
  }
  Serial.println(digitalRead(7));


}
