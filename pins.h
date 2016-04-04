#ifndef pins_h
#define pins_h
#include <Arduino.h>

//pins
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 6;
const int ArmBendPin = 7;
const int WristPin = 11;
const int GripPin = 10;
const int HallRgt = A0;
const int HallLft = A1;
const int GripLight = A2;
const int HallGrip = A3;
const int ci_I2C_SDA = A4;         // I2C data = white -> Nothing will be plugged into this
const int ci_I2C_SCL = A5;         // I2C clock = yellow -> Nothing will be plugged into this
const int UltrasonicPing = 2;//data return in 3
const int UltrasonicPingSide = 8;//data return in 9
//ULTRASONIC SIDE DATA RETURN ON D9

#endif
