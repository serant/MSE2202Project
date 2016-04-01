#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>
#include <PID_v1.h>
const unsigned long CourseWidth = 6000; //course width in mm
unsigned long XPos = 0;

//DEBUGGERS -> uncomment to debug
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS
//#define DEBUG_PID

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
unsigned LftSpeed = 0;
unsigned RgtSpeed = 0;
//Line Tracker Stuff
unsigned LineTrackerData = 0;
unsigned GripLightData = 0;

//Data variables
unsigned long HallSensorValue = 0;
unsigned long UltrasonicDistance = 0;

//PID Control
double targetSpeed, leftInput, rightInput, leftOutput, rightOutput;
double RightSpeed, RightPower, LeftSpeed;
double Kp = 11.9, Ki =100, Kd = 0.00001;
PID leftPid(&leftInput, &leftOutput, &targetSpeed, Kp, Ki, Kd, DIRECT);
PID rightPid(&rightInput, &rightOutput, &targetSpeed, Kp, Ki, Kd, DIRECT);
int k = 10;

PID motorPID(&RightSpeed, &RightPower, &LeftSpeed, Kp, Ki, Kd, DIRECT);
unsigned long prevTime = 0;
unsigned long currentTime = 0;
int i = 0;

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

//pins FINALIZED DO NOT CHANGE THIS///////////////////
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 26;
const int ArmBendPin = 27;
const int WristPin = 10;//********
const int GripPin = 11;//********
const int HallRgt = A0;
const int HallLft = A1;
const int GripLight = A2;
const int HallGrip = A3;//************
const int ci_I2C_SDA = A4;         // I2C data = white -> Nothing will be plugged into this 
const int ci_I2C_SCL = A5;         // I2C clock = yellow -> Nothing will be plugged into this
const int UltrasonicPing = 2;//data return in 3
//ULTRASONIC DATA RETURN ON D3
const int UltrasonicPingSide = 8;//data return in 9
//ULTRASONIC SIDE DATA RETURN ON D9

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
const int CE = 637;//pulses per revolution
const int CF = (3.14159 * 69.85) / CE; //Conversion factor, traslates encoder pulses to linear displacement

int DelLft = 0;
int DelRgt = 0;

int Dsp = 0;
int DelDsp = 0;
int PrvDsp = 0;
int FindDsp = 0;
int DspBuffer = 5;

int XPstn = 0;
int YPstn = 0;

int Theta = 0;
int FindTheta = 0;
int PickUpTheta = 0;
int ThetaBuffer = 2;

//Mode 1 Tesseract Placement Variables
int StepIndex = 1;
bool HitBlack = false;
const int GripLightDark = 0;
int HitBlackCount = 0;
int HitBlackTarget = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Set up two motors
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  // Set up encoders DO NOT CHANGE ORDER
  RgtEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward

  LftEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward

  //pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin); // 37 folded, 180 out

  //pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin); // 180 folded, 0 out
  pinMode(7, INPUT);

  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicPing + 1, INPUT);
  pinMode (UltrasonicPingSide, OUTPUT);
  pinMode(UltrasonicPingSide + 1, INPUT);

  HallIdle = (analogRead(HallLft) + analogRead(HallRgt) / 2); ///*********works???
  
  leftPid.SetMode(AUTOMATIC);
  rightPid.SetMode(AUTOMATIC);
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(1570,1830);
  motorPID.SetSampleTime(10);
}
void loop() {
  //WHATEVER IS IN THIS LOOP MUST BE OVERWRITTEN BY THE MASTER
  DebuggerModule();
  motorAccelerate(1700);
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

#ifdef DEBUG_PID
  if((millis() - prevTime) >=12){
    prevTime = millis();
    Serial.print("Left Input: ");
    Serial.println(LeftSpeed);
    Serial.print("Current Right Speed: ");
    Serial.println(RightSpeed);
    Serial.print("Current Right Power: ");
    Serial.println(RightPower);
  }
#endif
}

void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) / 58);
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
    if (turn == 1) turn = 3; // tess in middle
    else turn = 2;  // tess to right
  }
  switch (turn) {
    case 1:
      RgtMtr.write(1450); ///this should align robot a bit to left  *******test #s
      LftMtr.write(1400);
      delay(500);
      LftMtr.write(1800);
      RgtMtr.write(1800);
      delay(500);
      LftMtr.write(1600);
      RgtMtr.write(1600);
      break;
    case 2:
      LftMtr.write(1450); ///should align robot bit to right **********test #s
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
        ////******want to use IR or some other form of distance? think it may work better, especially for small distance
        Ping(UltrasonicPingSide);
        LftMtr.write(1600);
        RgtMtr.write(1400);
        break;
        UltrasonicDistance = 0;
        LftMtr.write(1500);
        RgtMtr.write(1500);

        Grip.write(/*open*/100);  /////pick up tesseract *********test #s
        Wrist.write(/*angled*/100);  //******* test #s
        ArmBase.write(110);      // 37 folded, 180 out
        ArmBend.write(150);    //180 folded, 0 out
        delay(500);
        Grip.write(/*closed*/0);   ///*********  test #s
        delay(500);
        ArmBase.write(40);
        ArmBend.write(160);

      }
  }
}

void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
};
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
  
  if(((Theta < (FindTheta - ThetaBuffer)) || Theta > (FindTheta + ThetaBuffer)) && ((Dsp < (FindDsp - DspBuffer)) || Dsp > (FindDsp + DspBuffer)))
  {
    LeftMotorSpeed = 1400;
    RightMotorSpeed = 1600;
  }
  
  else if(((Theta > (FindTheta - ThetaBuffer)) || Theta < (FindTheta + ThetaBuffer)) && ((Dsp < (FindDsp - DspBuffer)) || Dsp > (FindDsp + DspBuffer)))
  {
    LeftMotorSpeed = MotorSpeed + LeftMotorOffset;
    RightMotorSpeed = MotorSpeed + RightMotorOffset;
  }
  
  else if(((Theta > (FindTheta - ThetaBuffer)) || Theta < (FindTheta + ThetaBuffer)) && ((Dsp > (FindDsp - DspBuffer)) || Dsp < (FindDsp + DspBuffer)))
  {
    //switch control signal to go back to Look();
  }
  
}
void Position() {

}

void PlaceTesseract(){
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
  switch(StepIndex){
    case 1:
      ArmBend.write(0);
      ArmBase.write(0);
      Wrist.write(0);
      
      if(Theta < 200){
        RightMotorSpeed = 1600;
        LeftMotorSpeed = 1400;
      }
      else{
        StepIndex = 2;
      }
    break;
    
    case 2:
      RightMotorSpeed = 1600;
      LeftMotorSpeed = 1400;
      if((GripLightData < GripLightDark) && (!HitBlack)){
        HitBlackCount++;
        HitBlack = true;
        if(HitBlackCount == HitBlackTarget){
          StepIndex = 3;
        }
      }
      
      else if((GripLightData > GripLightDark) && (HitBlack)){
        HitBlack = false;
      }
    break;
    
    case 3:
      LeftMotorSpeed = 1500;
      RightMotorSpeed = 1500;
      ArmBend.write(0);
      ArmBase.write(0);
      Wrist.write(0);
      Grip.write(0);
      HitBlackCount = 0;
      HitBlackTarget--;
    break;
  }
}
//Mode 2
void Check() {
  //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
  // Robo --> back and forth scanning motion
  LeftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
  RightMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
  int lastHallReading = analogRead(HallGrip);
  int LftEncoderCounter = LftEncdr.getRawPosition();
  int RgtEncoderCounter = RgtEncdr.getRawPosition();

  LeftMotorSpeed = 1650;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter < 50; LftEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Forward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LeftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Backward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LeftMotorSpeed = 1500;
  LftMtr.writeMicroseconds(LeftMotorSpeed);
  delay(200);


  RightMotorSpeed = 1650;
  RgtMtr.writeMicroseconds(RightMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter < 50; RgtEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Forward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  RightMotorSpeed = 1350;
  RgtMtr.writeMicroseconds(RightMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Backward: ");
    Serial.println(RgtEncoderCounter);
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
void motorAccelerate(unsigned uSSpeed){
  
  for(k; k > 1; k--){
    motorPID.SetSampleTime(10);
    PIDSpeed(constrain((1500+((uSSpeed-1500)/k)), 1500, 2100));
    Serial.println(constrain((1500+((uSSpeed-1500)/k)), 1500, 2100));
  }
  motorPID.SetSampleTime(10);
  PIDSpeed(uSSpeed);
}
void PIDSpeed(unsigned uSSpeed){

  LeftSpeed = LftEncdr.getSpeed();
  RightSpeed = RgtEncdr.getSpeed();
  
  motorPID.Compute();
  
  LftMtr.writeMicroseconds(uSSpeed);
  RgtMtr.writeMicroseconds(RightPower);
}


//requires timer system and tesseracts picked up counter


