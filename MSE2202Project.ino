#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>
#include "PID_v1.h"
const unsigned CourseWidth = 6000; //course width in mm
unsigned long XPos = 0;

//Testing Variables 
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;
unsigned long testTime = 0;
unsigned long timerStart;
unsigned long timer;
unsigned tempEncoderPosition = 0;
//DEBUGGERS -> uncomment to debug
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS
//#define DEBUG_TRACKING
//#define DEBUG_PID

//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool StartTracking = false;
bool TurnRight = true;
bool startTask = true;

//Hall Sensor Stuff
#define NOFIELD 505L
#define NOFIELDLFT  503L
#define NOFIELDRGT 512L
#define NOFIELDGRIP 513L
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
I2CEncoder ArmBaseEncdr;
I2CEncoder ArmBendEncdr;

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

//pins FINALIZED DO NOT CHANGE THIS///////////////////
const int LftMtrPin = 5;
const int RgtMtrPin = 4;
const int ArmBasePin = 26;
const int ArmBendPin = 27;
const int WristPin = 210;//********
const int GripPin = 211;//********
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
int Stop = 1500;

// variables
unsigned int MotorSpeed;
unsigned int LftMotorSpeed;
unsigned int RgtMotorSpeed;
unsigned int LftMotorPos;
unsigned int RgtMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;

//PID Control
double PIDRgt, PIDRgtPwr, PIDLft;//monitored value, controlled value, setpoint
double Kp = 11.9, Ki = 100, Kd = 0.00001; //PID parameters
unsigned accSpd = 0;//used for acceleration of the robot to allow PID to operate properly
PID mtrPID(&PIDRgt, &PIDRgtPwr, &PIDLft, Kp, Ki, Kd, DIRECT);//PID control to allow robot to drive straight

// Tracking Variables
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
double PolTheta = 0;
double FindTheta = 0;
double PickUpTheta = 0;
double XPstn = 1;
double YPstn = 0;
double SvdDelDisp = 0;
unsigned targetTheta = 0; //used for reorienting robot
double savedLftEncdr = 0;
double LftEncdrCount = 0;
double savedLftEncdrReturn = 0;


int StepIndex;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
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

  HallIdle = (analogRead(HallLft) + analogRead(HallRgt) / 2); ///*********works???

  mtrPID.SetMode(AUTOMATIC);
  mtrPID.SetOutputLimits(1570, 1830);
  mtrPID.SetSampleTime(10);

  //ArmBase.write(40);
  //ArmBend.write(10);

}
void loop() {
  DebuggerModule();
  Position();
  timer = millis() / 1000; //time in seconds

  if (timer % 30 < 1) {
    Serial.print("time = ");
    Serial.println(timer);
  }
  if (timer >= 240) {  //4 min time limit
    Serial.println("Look: GoHome called");
    GoHome();       
    ModeIndex = 0;
  }
  if (startTask) {       //only runs this until mode started, 1 sec delay, must reset to change mode
    if (digitalRead(13)) {   //switch 1 up = stay in 0, down = start mode 1 or 2
      startTask = false;
      timerStart = millis();
      if (digitalRead(12)) ModeIndex = 2; // switch 3 and 1 on (down)
      else  ModeIndex = 1; // switch 3 off (up), 1 on (down)
      delay(1000);
    }
    else ModeIndex = 0; // switch 1 off(up), select mode then turn switch 1 on(down) when want to start
  }
  if (!(digitalRead(13))) ModeIndex = 0;

  switch (ModeIndex) {
    case 0: /***********sitting around waiting, use this mode to test stuff, then clear*/ 
    Serial.println("Main Loop: In Mode 0");


      break;

    case 1: /**********************************mode 1 base = look */ 
      Serial.println("Main Loop: In mode 1");

      //Looks for Blocks
      if((analogRead(HallLft) - NOFIELDLFT) > HallThreshold) GoHome(); //if tesseract is at left hall sensor, call pickup and pass 1 to indicate left -> REPLACE WITH PICKUP 1
      else if((analogRead(HallRgt) - NOFIELDRGT) > HallThreshold) GoHome();// -> REPLACE WITH PICKUP 0

      //Pings to detect if wall is in front
      Ping(UltrasonicPing);
      if(UltrasonicDistance <= 20){//if wall is 10cm or closer
        Serial.print("turning  ");
        Serial.println(UltrasonicDistance);
        RgtMtr.writeMicroseconds(Stop);//stops to prepare for turn
        LftMtr.writeMicroseconds(Stop);
        delay(100);

        if (TurnRight) {//if turning right...
          tempEncoderPosition = LftEncdr.getRawPosition();
          while ((LftEncdr.getRawPosition() < tempEncoderPosition + 980)) {
            mtrPID.SetMode(MANUAL);
            LftMtr.writeMicroseconds(1700);
            RgtMtr.writeMicroseconds(1500);
          }
          LftMtr.writeMicroseconds(1500);
          RgtMtr.writeMicroseconds(1500);
          delay(3000);
          mtrPID.SetMode(AUTOMATIC);
        }
        else {//if turning left...
          tempEncoderPosition = RgtEncdr.getRawPosition();
          while (RgtEncdr.getRawPosition() < (tempEncoderPosition + 980)) {
            mtrPID.SetMode(MANUAL);
            RgtMtr.writeMicroseconds(1800);
            LftMtr.writeMicroseconds(1500);
          }
          RgtMtr.writeMicroseconds(1500);
          LftMtr.writeMicroseconds(1500);
          delay(3000);
          mtrPID.SetMode(AUTOMATIC);
        }
        TurnRight = !TurnRight;
      }
      WriteForwardSpeed(1700);
      break;

    /*case 2:  /********************mode 2 base = check    Serial.println("In mode 2");
      //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'

      // Robo --> back and forth scanning motion
      lastHallRead = analogRead(HallGrip);
      lftEncoderCounter = LftEncdr.getRawPosition();
      rgtEncoderCounter = RgtEncdr.getRawPosition();

      ArmBase.write(100); // 37 - 179 folded to out
      ArmBend.write(110); // 0 -180 out to folded

      LftMtr.writeMicroseconds(1650);
      for (lftEncoderCounter; lftEncoderCounter < 40; lftEncoderCounter++) {
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        Serial.print("Left Encoder Forward: ");
        Serial.println(lftEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      LftMtr.writeMicroseconds(1500);
      for (lftEncoderCounter; lftEncoderCounter > 0; lftEncoderCounter--) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Left Encoder Backward: ");
        Serial.println(lftEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      LftMtr.writeMicroseconds(1500);
      delay(200);

      RgtMtr.writeMicroseconds(1650);
      for (rgtEncoderCounter; rgtEncoderCounter < 40; rgtEncoderCounter++) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Right Encoder Forward: ");
        Serial.println(rgtEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      RgtMtr.writeMicroseconds(1500);
      for (rgtEncoderCounter; rgtEncoderCounter > 0; rgtEncoderCounter--) {
        currentHallRead = analogRead(HallGrip);
        Serial.print("Right Encoder Backward: ");
        Serial.println(rgtEncoderCounter);
        if ((currentHallRead - lastHallRead > 15) || (currentHallRead - lastHallRead < -15)) {
          Move();
        }
      }
      RgtMtr.writeMicroseconds(1500);
      delay(200);

      break;*/

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

#ifdef DEBUG_TRACKING
  Serial.print("Displacement: ");
  Serial.print(Dsp);
  Serial.print("   , Polar Angle: ");
  Serial.print(PolTheta);
  Serial.print("   , Orientation Angle: ");
  Serial.println(OrTheta);

  Serial.print("Cartesian: ");
  Serial.print(XPstn);
  Serial.print(", ");
  Serial.println(YPstn);

  Serial.print("Instantaneous: ");
  Serial.print(DelDsp);
  Serial.print(", ");
  Serial.print(dTheta);
  Serial.print(" Deg");
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

#ifdef DEBUG_PID
  if ((millis() - prevTime) >= 12) {
    prevTime = millis();
    Serial.print("Left Input: ");
    Serial.println(PIDLft);
    Serial.print("Current Right Speed: ");
    Serial.println(PIDRgt);
    Serial.print("Current Right Power: ");
    Serial.println(PIDRgtPwr);
  }
#endif
}

void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);
  mtrPID.SetMode(MANUAL);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  mtrPID.SetMode(AUTOMATIC);
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) / 58);
  Serial.print("Ultrasonic distance: ");
  Serial.println(UltrasonicDistance);
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
void Countermeasures() {
  //robot reacts to interference by other robot, after safe returns to 'Look'
}
unsigned HallLftRead, HallRgtRead;
int turn;
void PickUp(unsigned side) {
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
      WriteForwardSpeed(1700);
      delay(500);
      WriteForwardSpeed(1600);
      break;
    case 2:
      LftMtr.write(1450); ///should align robot bit to right **********test #s
      RgtMtr.write(1400);
      delay(500);
      WriteForwardSpeed(1700);
      delay(500);
      WriteForwardSpeed(1600);
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

void Position() {
  // PickUpTheta, FindTheta, SvdRgtEncdr, SvdLftEncdr
  // Distance travelled
  DelRgt = (CF * ((RgtEncdr.getRawPosition()))); // Instantaneous Distance traveled by right Wheel
  DelLft = (CF * ((LftEncdr.getRawPosition()))); // Instantaneous Distnace traveled by left wheel
  DelDsp = (DelRgt + DelLft) / 2; //Distance traveled by the centerpoint of the robot, affected by quadrent
  Dsp = Dsp + DelDsp; //Current Displacement
  TotalDsp = (abs(DelRgt) + abs(DelLft)) / 2; //Total distance travelled
  //Serial.print("Displacement: ");
  //Serial.println(Dsp);

  OrTheta = ((DelRgt - DelLft) / 115.5) * (180 / PI); // Change in orientation, taking starting postion as Theta = 0
  OrTheta = (int)OrTheta % 360; //If the magnitude of the orientation is greater than 360

  Serial.print("Orientation Theta: ");
  Serial.println(OrTheta); // Theta from wherever the bot was first placed

  XPstn = DelDsp * cos((OrTheta * PI) / 180);
  YPstn = DelDsp * sin((OrTheta * PI) / 180);
  //Serial.print("X: ");  //X coordinates of the robot (right is positive)
  //Serial.println(XPstn);
  //Serial.print( "Y: ");  //Y coordinates of the robot (up is positive)
  //Serial.println(YPstn);
  PolTheta = (atan(YPstn / XPstn) * (180 / PI)); //The polar angle of the position of the robot
  Serial.print("Pol Theta: ");
  Serial.println(PolTheta);
}

void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'

  Position();
  for (int i = 0; i > 0; i++) {
    SvdDsp = DelDsp;
    PickUpTheta = PolTheta;
    Serial.println("saved values");
  }

  if (!TurnRight) { //Turn number is even
    targetTheta = PolTheta;
    while (!(OrTheta < (targetTheta + 185) && OrTheta > (targetTheta + 175))) {
      Serial.println("Alinging Bot, even turn...");
      LftMtr.write(1350);
      RgtMtr.write(1650);
      Position();
    }
  }
  else { // Turn number is odd
    targetTheta = PolTheta;
    while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta - 5))) { //Test
      Serial.println("Alinging Bot, odd turn...");
      LftMtr.write(1650);
      RgtMtr.write(1350);
      Position();
    }
  }

  LftMtr.write(1500);
  RgtMtr.write(1500);
  //delay(500);
  Ping(2);

  savedLftEncdr = abs(LftEncdr.getRawPosition());
  //savedRgtEncdr = abs(RgtEncdr.getRawPosition());
  while ((UltrasonicDistance > 10) && (UltrasonicDistance != 0)) {

    Serial.println("Moving towards origin...");
    Serial.print(UltrasonicDistance * 58, DEC);
    WriteForwardSpeed(1700);
    Position();
    Ping(2);
  }
  LftEncdrCount = abs(LftEncdr.getRawPosition()) - savedLftEncdr;
  //RgtEncdrCount = abs(Rgt.Encdr.getRawPosition()) - savedRgtEncdr;
  //TravelEncdrCount = (LftEncdrCount + RgtEncdrCount )/2;

  LftMtr.write(1500);
  RgtMtr.write(1500);
  Return();
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
  Serial.println(PickUpTheta);
  while (!(OrTheta < (PickUpTheta +5) && OrTheta > (PickUpTheta - 5))) { // was +5 and -5
    Serial.println("Alinging Bot with saved polar theta...");
    Serial.println(OrTheta);
    LftMtr.write(1650);
    RgtMtr.write(1350);
    Position();
  }

  LftMtr.write(1500);
  RgtMtr.write(1500);

  savedLftEncdrReturn = abs(LftEncdr.getRawPosition());
  //savedRgtEncdrReturn = abs(RgtEncdr.getRawPosition());
  while (abs(LftEncdr.getRawPosition()) < (abs(savedLftEncdrReturn) + LftEncdrCount)) {
    Serial.println("Moving towards pickup position... ");
    WriteForwardSpeed(1700);
    Position();
  }
  /*
    SvdDelDisp = ((DelRgt + DelLft) / 2) + sqrt((XPstn * XPstn) + (YPstn * YPstn));
    Serial.print("Saved Disp: ");
    Serial.println(SvdDelDisp);
    while ((((DelRgt) + (DelLft)) / 2) < SvdDelDisp) { //Check
    Serial.println("Moving towards pickup position... ");
    Serial.println(((((DelRgt) + (DelLft)) / 2) < SvdDelDisp));
    WriteForwardSpeed(1700);
    Position();
    }
  */
  LftMtr.write(1500);
  RgtMtr.write(1500);

  if (!TurnRight) { //Turn number is even
    while (!(OrTheta < 5 && OrTheta > -5)) {
      Serial.println("Alinging Bot with 0 degrees, even turn...");
      LftMtr.write(1350);
      RgtMtr.write(1650);
      Position();
    }

  } else { // Turn number is odd
    while (!(OrTheta < 185 && OrTheta > 175)) {
      Serial.println("Alinging Bot with 180 degrees, odd turn...");
      LftMtr.write(1350);
      RgtMtr.write(1650);
      Position();
    }
  }
}

void PlaceTesseract() {
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
  switch (StepIndex) {
    case 1:
      ArmBend.write(0);
      ArmBase.write(0);
      Wrist.write(0);

      if (OrTheta < 200) {
        RgtMotorSpeed = 1600;
        LftMotorSpeed = 1400;
      }
      else {
        StepIndex = 2;
      }
      break;

    case 2:
      RgtMotorSpeed = 1600;
      LftMotorSpeed = 1400;
      if ((GripLightData < GripLightDark) && (!HitBlack)) {
        HitBlackCount++;
        HitBlack = true;
        if (HitBlackCount == HitBlackTarget) {
          StepIndex = 3;
        }
      }

      else if ((GripLightData > GripLightDark) && (HitBlack)) {
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

  ArmBase.write(100); // 37 - 179 folded to out
  ArmBend.write(110); // 0 -180 out to folded

  LftMotorSpeed = 1650;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter < 40; LftEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
    Serial.print("Left Encoder Forward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Left Encoder Backward: ");
    Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  LftMotorSpeed = 1500;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  delay(200);

  RgtMotorSpeed = 1650;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter < 40; RgtEncoderCounter++) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Forward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  RgtMotorSpeed = 1350;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  for (RgtEncoderCounter; RgtEncoderCounter > 0; RgtEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    Serial.print("Right Encoder Backward: ");
    Serial.println(RgtEncoderCounter);
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  RgtMotorSpeed = 1500;
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  delay(200);
}

void Move() {
  //robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'
  bool WallDistance = false;
  int GripCounter;
  int DriveStraight = false;
  int FirstValue;
  int SecondValue;
  int StraightCount = false;


  while (WallDistance == false) { // approach wall
    Ping(UltrasonicPing);
    if (UltrasonicDistance > 21) {
      WriteForwardSpeed(1600);
      LftMtr.writeMicroseconds(LftMotorSpeed);
      RgtMtr.writeMicroseconds(RgtMotorSpeed);
    }
    if (UltrasonicDistance < 17) {
      LftMtr.writeMicroseconds(1500);
      RgtMtr.writeMicroseconds(1500);
      WallDistance = true;
    }
  }
  // Robot picks up tesseract
  Grip.write(90); // open grip
  delay(300);
  ArmBend.write(165); // extend arm
  ArmBase.write(165);
  delay(300);

  while (analogRead(GripLight) < 950) { // 950 --> light, over 1000 --> dark
    LftMotorSpeed = 1425;
    LftMtr.writeMicroseconds(1425);
  }
  Grip.writeMicroseconds(150); // close grip

  RgtMotorSpeed = 1350; // back
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  RgtMtr.writeMicroseconds(RgtMotorSpeed);
  delay(400);
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  LftMotorSpeed = 1650; // turn right towards overhang
  LftMtr.writeMicroseconds(LftMotorSpeed);
  delay(200);
  LftMtr.writeMicroseconds(1500);

  while (DriveStraight == false) {
    Ping(2);
    if (StraightCount == false) {
      FirstValue = UltrasonicDistance;
    }
    SecondValue = UltrasonicDistance;

    if (SecondValue - FirstValue > 2) {
      RgtMtr.writeMicroseconds(1650);
      delay(50);
    }
    if (SecondValue - FirstValue > -2) {
      LftMtr.writeMicroseconds(1650);
      delay(50);
    }
    StraightCount = true;
    FirstValue = SecondValue;
  }
}
void DropOff() {
  ArmBend.writeMicroseconds(90); // extend arm uwards
  ArmBase.writeMicroseconds(90);
  Wrist.write(0); // 0 --> max upwards, 180 --> max downwards
  delay(1000);

  while (analogRead(2) > 950) { // over 1000 --> light, less than 500 -->dark
    LftMtr.writeMicroseconds(1350);
    RgtMtr.writeMicroseconds(1350);
  }
  Grip.writeMicroseconds(90); //  open grip
}

//PID FUNCTIONS
//THIS IS HOW YOU WRITE A FORWARD SPEED TO THE ROBOT.
//IT'S THE EXACT SAME AS servoObject.write(pwmSpd)
//pwmSpd -> desired pwm in ms
//servoObject -> either LftMtr or RgtMtr
void WriteForwardSpeed(unsigned pwmSpd) {
  //If the robot hasn't reached the desired speed, keep accelerating
  if (LftMtr.readMicroseconds() != pwmSpd) { //stops when desired speed is written to LftMtr
    MotorAccelerate(pwmSpd);
  }
  else {
    //Serial.println("begin coasting");
    PIDSpeed(pwmSpd);//if robot has reached desired speed, keep speed
  }
}
void MotorAccelerate(unsigned uSSpd) {
  for (int accStps = 10; accStps >= 1; accStps--) { //steps to accelerate robot
    mtrPID.SetSampleTime(10);//change this value if the robot moves off track at the beginning
    accSpd = constrain((1500 + ((uSSpd - 1500) / accStps)), 1500, 2100); //left speed increases from 1500ms to target speed
    PIDSpeed(accSpd);//sends the current speed for PID control to right motor
  }
  mtrPID.SetSampleTime(10);//sets sampling time for PID control
  PIDSpeed(uSSpd);//sets first datapoint for target speed
  //Serial.println("Finished accelerating");
}
void PIDSpeed(unsigned uSSpd) { //used to ensure robot travels straight during constant velocity
  PIDLft = LftEncdr.getSpeed();//set point
  PIDRgt = RgtEncdr.getSpeed();//monitored variable

  mtrPID.Compute();//computes using Kp, Ki, Kd

  LftMtr.writeMicroseconds(uSSpd);//writes desired pwm pulse to left motor
  RgtMtr.writeMicroseconds(PIDRgtPwr);//writes controled pwm pulse to right motor
}

