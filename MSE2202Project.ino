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
unsigned LineTrackerData = 0;
unsigned GripLightData = 0;
unsigned GripLightDark = 0;
bool HitBlack = false;
int HitBlackCount = 0;
int HitBlackTarget = 3;

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
unsigned int ModeIndex = 4;
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
const int WristPin = 0;//********
const int GripPin = 0;//********
const int HallRgt = A5;
const int HallLft = A4;
const int HallGrip = A0;//************
const int GripLight = A2;
const int UltrasonicPing = 2;//data return in 3
const int UltrasonicPingSide = 8;//data return in 9

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
long RawLftPrv = 0;
long RawRgtPrv = 0;
const double CE = 637;//pulses per revolution 
const double CF= ((3.14159*69.85)/CE); //Conversion factor, traslates encoder pulses to linear displacement
double DelLft = 0;
double DelRgt = 0;
double DelDsp = 0; 
long TotalDsp = 0;
double SvdDsp = 0;
double Dsp = 1000;
double OrTheta = 0;
double PrvOrTheta = 0;
double dTheta = 0;
double PolTheta = 105;
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

  // Set up two motors
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

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

  HallIdle = (analogRead(HallLft) + analogRead(HallRgt) / 2); ///*********works???

}
void loop() {
  DebuggerModule();
  int timer = millis();
  
  //Temporary code for tracking testing=============================================//

  switch(ModeIndex){
    case 1:
      Look();
    break;
    
    case 2:
      Countermeasures();
    break;
    
    case 3:
      PickUp();
    break;
    
    case 4:
      GoHome();
    break;
    
    case 5:
      Return();
    break;
    
    case 6:
      Check();
    break;
    
    case 7:
      Move();
    break;
    
  }

  Position();

  //NONE OF THE BELOW SHOULD BE OUTSIDE OF THE SWITCH STATEMENT SO WE NEED TO ORGANIZE THIS 
  //if(timer > 8000){
  //GoHome();
  //}

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

  Look();
  if (StartTracking) {

    TrackPosition();
  }*/
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

  LftMotorPos = LftEncdr.getRawPosition();
  RgtMotorPos = RgtEncdr.getRawPosition();

  Serial.print("Encoders L: ");
  Serial.print(LftMotorPos);
  Serial.print(", R: ");
  Serial.println(RgtMotorPos);

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

void Position(){
  // PickUpTheta, FindTheta, SvdRgtEncdr, SvdLftEncdr

  // Distance travelled 
  DelRgt = (CF * ((RgtEncdr.getRawPosition()) - RawRgtPrv)); // Instantaneous Distance traveled by right Wheel 
  DelLft = CF * ((LftEncdr.getRawPosition()- RawLftPrv)); // Instantaneous Distnace traveled by left wheel 
  DelDsp = (DelRgt + DelLft)/2; //Instantaneous Distance traveled by the centerpoint of the robot
  Dsp = Dsp + DelDsp; //Current Displacement 
  Serial.print("Displacement: ");
  Serial.println(Dsp);
  
  dTheta = ((DelRgt - DelLft)/109) *(180/PI); // Change in orientation, taking starting postion as Theta = 0
  OrTheta = OrTheta + dTheta; //Orientation of robot
  OrTheta = (int)OrTheta%360; //If the magnitude of the orientation is greater than 360

  Serial.print("Orientation Theta: ");
  Serial.println(OrTheta);
  
  dXPstn = DelDsp * cos(OrTheta*PI/180);
  dYPstn = DelDsp * sin(OrTheta*PI/180);
  XPstn = XPstn + dXPstn;
  YPstn = YPstn + dYPstn;
  Serial.print("X: ");//X coordinates of the robot (right is positive)
  Serial.print(XPstn); 
  Serial.print( "Y: ");//Y coordinates of the robot (up is positive)
  PolTheta = atan(YPstn/XPstn*180/PI);//The polar angle of the position of the robot

  RawLftPrv = LftEncdr.getRawPosition();
  RawRgtPrv = RgtEncdr.getRawPosition();
  PrvOrTheta = OrTheta;
}

void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  Position();
  for (int i = 0; i>0; i++){
    
    SvdDsp = Dsp; 
  }
  while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta - 5))){
    Serial.println("Alinging Bot...");
    LftMtr.write(1500);
    RgtMtr.write(1300);
    Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
 
  while (Dsp > 10){
     Serial.println("Moving towards origin...");
     LftMtr.write(2000);
     RgtMtr.write(2000);
     Position();
  }
  LftMtr.write(1500);
  RgtMtr.write(1500);
  
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
  
  if(((OrTheta < (FindTheta - ThetaBuffer)) || OrTheta > (FindTheta + ThetaBuffer)) && ((Dsp < (SvdDsp - DspBuffer)) || Dsp > (SvdDsp + DspBuffer)))
  {
    LftMotorSpeed = 1400;
    RgtMotorSpeed = 1600;
  }
  
  else if(((OrTheta > (FindTheta - ThetaBuffer)) || OrTheta < (FindTheta + ThetaBuffer)) && ((Dsp < (SvdDsp - DspBuffer)) || Dsp > (SvdDsp + DspBuffer)))
  {
    LftMotorSpeed = MotorSpeed + LeftMotorOffset;
    RgtMotorSpeed = MotorSpeed + RightMotorOffset;
  }
  
  else if(((OrTheta > (FindTheta - ThetaBuffer)) || OrTheta < (FindTheta + ThetaBuffer)) && ((Dsp > (SvdDsp - DspBuffer)) || Dsp < (SvdDsp + DspBuffer)))
  {
    //switch control signal to go back to Look();
  }
  
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
      
      if(OrTheta < 200){
        RgtMotorSpeed = 1600;
        LftMotorSpeed = 1400;
      }
      else{
        StepIndex = 2;
      }
    break;
    
    case 2:
      RgtMotorSpeed = 1600;
      LftMotorSpeed = 1400;
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
      LftMotorSpeed = 1500;
      RgtMotorSpeed = 1500;
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
  LftMotorSpeed = constrain(MotorSpeed + LeftMotorOffset, 1500, 2200);
  RgtMotorSpeed = constrain(MotorSpeed + RightMotorOffset, 1500, 2200);
  int lastHallReading = analogRead(HallGrip);
  int LftEncoderCounter = LftEncdr.getRawPosition();
  int RgtEncoderCounter = RgtEncdr.getRawPosition();

  LftMotorSpeed = 1650;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter < 50; LftEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Forward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Backward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  LftMotorSpeed = 1500;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  delay(200);


  RgtMotorSpeed = 1650;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter < 50; RgtEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Forward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
      return;
    }
  }
  RgtMotorSpeed = 1350;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Backward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallReading - lastHallReading > 20) {
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


