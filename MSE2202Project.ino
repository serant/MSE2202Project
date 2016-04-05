//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS
//#define DEBUG_TRACKING
//#define DEBUG_PID

#include <CharliePlexM.h>
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>
#include <PID_v1.h>


//Testing Variables
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;
unsigned long testTime = 0;
unsigned tempEncoderPosition = 0;

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


//Flags/Switches
bool StartLooking = true;
bool EnableIncrement = true;
bool TurnRight = true;
bool StartTracking = false;
bool startTask = true;
bool start = true;
int AnyUse;
unsigned pickedUp;
bool Start = true;

//Hall Sensor Stuff
#define NOFIELDGRIP 510L
int NOFIELDRGT = 0;
int NOFIELDLFT = 0;
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  

int currentHallRead;
int lastHallRead;
unsigned HallThreshold = 6;

//Mechanical Information
unsigned WheelPerimeter = (69.85 * PI) / 10; //perimeter of wheel in cm
unsigned ForwardSpeed = 1700; //speed of robot while looking in mode 1
unsigned Stop = 1500;
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
unsigned int timer;
unsigned long timerStart;
unsigned timeRun = 5;



Servo LftMtr;
Servo ArmBend;    //out -> folded 0->180
Servo ArmBase;    //folded->out  37-180
Servo RgtMtr;
Servo Grip;       //170 closed, 100 open
Servo Wrist;      //0 min folded up, 50 straight out, 180 folded down
I2CEncoder LftEncdr;
I2CEncoder RgtEncdr;
I2CEncoder ArmBaseEncdr;
I2CEncoder ArmBendEncdr;

//Mode Selector Variables

unsigned int ModeIndex = 4;


//pins FINALIZED DO NOT CHANGE THIS///////////////////


int MovFst = 2200;

// variables
unsigned int MotorSpeed;
unsigned int LftMotorSpeed;
unsigned int RgtMotorSpeed;
unsigned int LftMotorPos;
unsigned int RgtMotorPos;
unsigned long LeftMotorOffset;
unsigned long RightMotorOffset;
long lftEncoderCounter;
long rgtEncoderCounter;

//PID Control
double PIDRgt, PIDRgtPwr, PIDLft;//monitored value, controlled value, setpoint
double Kp = 11.9, Ki = 100, Kd = 0.00001; //PID parameters
unsigned accSpd = 0;//used for acceleration of the robot to allow PID to operate properly
PID mtrPID(&PIDRgt, &PIDRgtPwr, &PIDLft, Kp, Ki, Kd, DIRECT);//PID control to allow robot to drive straight
unsigned long PIDTimer;


// Tracking Variables
unsigned long CourseWidth = 240; //course width in cm, has to be set prior to running
unsigned long XPos = 0;
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
double PrvOrTheta = 0;
double dTheta = 0;
double PolTheta = 0;
double FindTheta = 0;
double PickUpTheta = 0;
double XPstn = 0;
double dXPstn = 0;
double YPstn = 0;
bool Black = false;
bool Light = false;
int BlockNumber = 0;
int Line = 0;
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

  Grip.write(110);

  pinMode(WristPin, OUTPUT);
  Wrist.attach(WristPin);
  Wrist.write(80);

  // Set up encoders DO NOT CHANGE ORDER
  RgtEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward

  LftEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward

  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  ArmBase.write(40);

  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBend.write(150);

  pinMode(7, INPUT);

  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicPing + 1, INPUT);
  pinMode (UltrasonicPingSide, OUTPUT);
  pinMode(UltrasonicPingSide + 1, INPUT);

  mtrPID.SetMode(AUTOMATIC);
  mtrPID.SetOutputLimits(1500, 1900);
  mtrPID.SetSampleTime(10);
  
  for(int i = 0; i < 10; i++){
    NOFIELDLFT += analogRead(HallLft);
    NOFIELDRGT += analogRead(HallRgt);
    delay(100);
  }
  NOFIELDLFT = NOFIELDLFT/10;
  NOFIELDRGT = NOFIELDRGT/10;
}

void loop() {
  //***************************stuff running through every time
  DebuggerModule();
  Position();

  timer = millis() / 1000; //time in seconds
  //Ping(2);
  //Serial.println(UltrasonicDistance);
  if (timer % 5 < 1) {
    Serial.print("time = ");
   // Serial.println(timer);

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
      delay(3000);
    }
    else ModeIndex = 0; // switch 1 off(up), select mode then turn switch 1 on(down) when want to start
  }
  if (!(digitalRead(13))) ModeIndex = 0;

  switch (ModeIndex) {
    case 0: /***********sitting around waiting, use this mode to test stuff, then clear*/
    Serial.print("LEFT: ");
    Serial.println((analogRead(HallLft) - NOFIELDLFT) > 10);
    Serial.print("RIGHT: ");
    Serial.println((analogRead(HallRgt) - NOFIELDRGT) > 10);

      break; 

    case 1: /**********************************mode 1 base = look */
      Serial.println("Main Loop: In mode 1");

      //Looks for Blocks
      if ((analogRead(HallLft) - NOFIELDLFT) > HallThreshold) PickUp(1); //if tesseract is at left hall sensor, call pickup and pass 1 to indicate left 
      else if ((analogRead(HallRgt) - NOFIELDRGT) > HallThreshold) PickUp(0); //

      //Pings to detect if wall is in front
      Ping(UltrasonicPing);
      if (UltrasonicDistance <= 20) { //if wall is 10cm or closer
        Serial.print("turning  ");
        Serial.println(UltrasonicDistance);
        RgtMtr.writeMicroseconds(Stop);//stops to prepare for turn
        LftMtr.writeMicroseconds(Stop);
        delay(100);

      //Looks for Blocks
      if (((analogRead(HallLft) - NOFIELDLFT) > 10) ||((analogRead(HallLft) - NOFIELDLFT) < -10)) {//if tesseract is at left hall sensor, call pickup and pass 1 to indicate left
      Serial.println("entered left");
        PickUp(1);
      }
       //else if (((analogRead(HallRgt) - NOFIELDRGT) > 10) || ((analogRead(HallRgt) - NOFIELDRGT) < -10 )) {
        //PickUp(0);
       // Serial.println("Entered right");
      //}
      else if ((millis() - timerStart) > timeRun) {
        LftMtr.writeMicroseconds(1500);
        RgtMtr.writeMicroseconds(1500);
        mtrPID.SetMode(MANUAL);
        delay(500);
        mtrPID.SetMode(AUTOMATIC);
        Ping(UltrasonicPing);
        if (UltrasonicDistance < 300 && UltrasonicDistance != 0) timeRun = 4000;
        if (UltrasonicDistance < 80 && UltrasonicDistance != 0) timeRun = 1500;
        if (UltrasonicDistance < 50 && UltrasonicDistance != 0) timeRun = 500;
        if (UltrasonicDistance < 25 && UltrasonicDistance > 2) {
          RgtMtr.writeMicroseconds(Stop);//stops to prepare for turn
          LftMtr.writeMicroseconds(Stop);
          delay(100);
          if (TurnRight) {//if turning right...
            tempEncoderPosition = LftEncdr.getRawPosition();
            while ((LftEncdr.getRawPosition() < tempEncoderPosition + 960)) {
              LftMtr.writeMicroseconds(1700);
            }
            LftMtr.writeMicroseconds(1500);
            RgtMtr.writeMicroseconds(1500);
          }
          else {//if turning left...
            tempEncoderPosition = RgtEncdr.getRawPosition();
            while (RgtEncdr.getRawPosition() < (tempEncoderPosition + 960)) {
              RgtMtr.writeMicroseconds(1700);
            }
            RgtMtr.writeMicroseconds(1500);
            LftMtr.writeMicroseconds(1500);
          }
          TurnRight = !TurnRight;
          timeRun = 5;
        }
        timerStart = millis();
      }
      else {
        WriteForwardSpeed(1800);
      }

      break;

      case 2:  /********************mode 2 base = check   */ Serial.println("In mode 2");

      //robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
      // Robo --> back and forth scanning motion
      lastHallRead = analogRead(HallGrip);
      lftEncoderCounter = LftEncdr.getRawPosition();
      rgtEncoderCounter = RgtEncdr.getRawPosition();

      ArmBase.write(90); // 37 - 179 folded to out
      ArmBend.write(115); // 0 -180 out to folded
      Grip.write(170); // closed grip
      Wrist.write(100);
      delay(1000);

      while (UltrasonicDistance > 21 || UltrasonicDistance < 10) {
        for (int i = 0; i < 4; i++) {
          Ping(UltrasonicPing);
        }
        LftMtr.writeMicroseconds(1650);
        RgtMtr.writeMicroseconds(1660);
        delay(100);
        LftMtr.writeMicroseconds(1500);
        RgtMtr.writeMicroseconds(1500);
        delay(100);
      }
      LftEncdr.zero();
      RgtEncdr.zero();
      while (LftEncdr.getRawPosition() < 200) {
        LftMtr.writeMicroseconds(1700);
        delay(100);
        LftMtr.writeMicroseconds(1500);
        delay(100);
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -122)) {
          Move();
        }
      }
      delay(400);
      while (LftEncdr.getRawPosition() > 0) {
        LftMtr.writeMicroseconds(1330);
        delay(100);
        LftMtr.writeMicroseconds(1500);
        delay(100);
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
          Move();
        }
      }
      delay(400);
      while (RgtEncdr.getRawPosition() < 200) {
        RgtMtr.writeMicroseconds(1630);
        delay(100);
        RgtMtr.writeMicroseconds(1500);
        delay(100);
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
          Move();
        }
      }
      delay(400);
      while (RgtEncdr.getRawPosition() > 0) {
        RgtMtr.writeMicroseconds(1370);
        delay(100);
        RgtMtr.writeMicroseconds(1500);
        delay(100);
        currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
        if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
          Move();
        }
      }
      delay(400);
      break;
    }
  }
}
//any time functions
void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);
  mtrPID.SetMode(MANUAL);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  mtrPID.SetMode(AUTOMATIC);
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) * 1.1 / 58);//returns in cm
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

void PickUp(int i) {  //left = 1, right = 0
  //robot has deteced tesseract and uses arm to pick it up, after picked up runs 'GoHome'

  switch (i) {
    case 0:
    LftMtr.writeMicroseconds(1450);
    RgtMtr.writeMicroseconds(1400);
    delay(1000);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);
    delay(200);
    LftMtr.writeMicroseconds(1600);
    RgtMtr.writeMicroseconds(1600);
    delay(1000);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);

    i = 3;
    break;
    case 1:
    i = 3;
    break;
    case 3:
    delay(500);
    LftMtr.writeMicroseconds(1350);
    RgtMtr.writeMicroseconds(1350);
    delay(1500);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);
    i = 9;
  }
  delay(500);
  LftMtr.writeMicroseconds(1350);
  RgtMtr.writeMicroseconds(1350);
  delay(1500);
  LftMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(Stop);
  Grip.write(100);     //picking up tesseract
  Wrist.write(48);
  ArmBend.write(130);
  delay(1000);
  ArmBase.write(110);
  delay(1000);
  ArmBase.write(125);
  delay(1000);

  Grip.write(170);      //close grip and retract arm
  delay(1000);
  ArmBase.write(90);
  delay(1000);
  Serial.println(analogRead(HallGrip) - NOFIELDGRIP);
  if (!((analogRead(HallGrip) - NOFIELDGRIP) > 5 || ((analogRead(HallGrip) - NOFIELDGRIP) < -5))) { //drop tesseract if not magnetic/missed tesseract
    ArmBend.write(20);
    delay(1000);
    Grip.write(100);
    delay(500);
    for (int j = 20; j <= 130; j += 5) {
      ArmBend.write(j);
      delay(100);
    }
  }
  else {
    LftMtr.writeMicroseconds(1650);
    RgtMtr.writeMicroseconds(1650);
    delay(900);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);
  }
  Grip.write(170);
  delay(1000);
  ArmBase.write(90);
  delay(1000);
  int tempTime = millis();
  while((millis()-tempTime) <= 3000){
    WriteForwardSpeed(1700);
  }
  LftMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(Stop);
//  if (!((analogRead(HallGrip) - NOFIELDGRIP) > 8 || ((analogRead(HallGrip) - NOFIELDGRIP) < -8))) { //drop tesseract if not magnetic/missed tesseract
//    ArmBend.write(20);
//    delay(1000);
//    Grip.write(100);
//    delay(500);
//    for (int j = 20; j <= 130; j += 5) {
//      ArmBend.write(j);
//      delay(100);
//    }
//  }
//  else {
     Serial.println("got tesseract");

  GoHome();
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

 // Serial.print("Orientation Theta: ");
  //Serial.println(OrTheta); // Theta from wherever the bot was first placed

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
    //Serial.println("saved values");
  }

  if (!TurnRight) { //Turn number is even, right turn == true
    targetTheta = PolTheta;
    while (!(OrTheta < (targetTheta + 185) && OrTheta > (targetTheta + 175))) {
      Serial.println("Alinging Bot, even turn...");
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);
      Position();
    }
  }
  else { // Turn number is odd
    targetTheta = PolTheta;
    while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta -5))) { //Test
      Serial.println("Alinging Bot, odd turn...");
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);

      Position();
    }
  }

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  //delay(500);
  Ping(2);

  savedLftEncdr = abs(LftEncdr.getRawPosition());
  //savedRgtEncdr = abs(RgtEncdr.getRawPosition());
  while ((UltrasonicDistance > 19) && (UltrasonicDistance != 0)) {

    //Serial.println("Moving towards origin...");
    //Serial.print(UltrasonicDistance * 58, DEC);
    WriteForwardSpeed(1700);
    Position();
    Ping(2);
  }
  LftEncdrCount = abs(LftEncdr.getRawPosition()) - savedLftEncdr;
  //RgtEncdrCount = abs(Rgt.Encdr.getRawPosition()) - savedRgtEncdr;
  //TravelEncdrCount = (LftEncdrCount + RgtEncdrCount )/2;

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  PlaceTesseract();
}

void Return() {
    Position();
    Serial.println(PickUpTheta);
  while (!(OrTheta < (PickUpTheta + 5) && OrTheta > (PickUpTheta - 5))) { // was +5 and -5
    Serial.println("Alinging Bot with saved polar theta...");
    Serial.println(OrTheta);
    LftMtr.writeMicroseconds(1650);
    RgtMtr.writeMicroseconds(1350);

    Position();
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  savedLftEncdrReturn = abs(LftEncdr.getRawPosition());
  //savedRgtEncdrReturn = abs(RgtEncdr.getRawPosition());
  while (abs(LftEncdr.getRawPosition()) < (abs(savedLftEncdrReturn) + LftEncdrCount)) {
    //Serial.println("Moving towards pickup position... ");
    WriteForwardSpeed(1700);
    Position();
  }
  
    SvdDelDisp = ((DelRgt + DelLft) / 2) + sqrt((XPstn * XPstn) + (YPstn * YPstn));
    Serial.print("Saved Disp: ");
    Serial.println(SvdDelDisp);
    while ((((DelRgt) + (DelLft)) / 2) < SvdDelDisp) { //Check
    Serial.println("Moving towards pickup position... ");
    Serial.println(((((DelRgt) + (DelLft)) / 2) < SvdDelDisp));
    WriteForwardSpeed(1700);
    Position();
    }
    
    LftMtr.writeMicroseconds(1500);
    RgtMtr.writeMicroseconds(1500);

  if (!TurnRight) { //Turn number is even
    while (!(OrTheta < 5 && OrTheta > -5)) {
      Serial.println("Alinging Bot with 0 degrees, even turn...");
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);

      Position();
    }

  } else { // Turn number is odd
    while (!(OrTheta < 185 && OrTheta > 175)) {
      Serial.println("Alinging Bot with 180 degrees, odd turn...");
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);

      Position();
    }
  }
}

void PlaceTesseract() {

  // Allign with wall
  while (!((OrTheta < -210) && (OrTheta > -220))) {
    LftMtr.writeMicroseconds(1625);
    RgtMtr.writeMicroseconds(1375);
    //Serial.println(OrTheta);
    Position();
  }
  // Move towards wall
  Ping(2);
  while (UltrasonicDistance < 17 && UltrasonicDistance != 0) {
    LftMtr.writeMicroseconds(1350);
    RgtMtr.writeMicroseconds (1350);
    Ping(2);
  }

  while (UltrasonicDistance > 21 && UltrasonicDistance != 0) {
    LftMtr.writeMicroseconds (1650);
    RgtMtr.writeMicroseconds(1650);
    Ping(2);
  }

  // Turn towards orientation Theta
  while (!(OrTheta < 5 && OrTheta > -5 )) {
    LftMtr.write(1700);
    RgtMtr.write(1300);
    Position();
  }
}
 

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
    //Serial.print("Left Encoder Forward: ");
    //Serial.println(LftEncoderCounter);
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
    //Serial.print("Left Encoder Backward: ");
    //Serial.println(LftEncoderCounter);
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
    //Serial.print("Right Encoder Forward: ");
    //Serial.println(RgtEncoderCounter);
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


void Move() {//detected tesseract on wall, pick it up, turn, move under beam, then run DropOff
  //robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'

  LftMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(1330);
  delay(150);
  RgtMtr.writeMicroseconds(1500);
  delay(100);
  Grip.write(100);
  delay(700);
  ArmBend.write(120); // drop arm around tesseract
  ArmBase.write(90);
  delay(700);
  LftMtr.writeMicroseconds(1650);
  RgtMtr.writeMicroseconds(1650);
  delay(200);
  LftMtr.writeMicroseconds(Stop);
  Grip.write(170); // close grip
  delay(1000);
  ArmBend.write(160);
  ArmBase.write(40);
  Serial.println("Picked up the tesseract!");
  while (LftEncdr.getRawPosition() > 0) {     //alligning back to center
    LftMtr.writeMicroseconds(1350);
  }
  while (LftEncdr.getRawPosition() <= 0) {
    LftMtr.writeMicroseconds(1650);
  }
  LftMtr.writeMicroseconds(1500);
  while (RgtEncdr.getRawPosition() > 0) {
    RgtMtr.writeMicroseconds(1350);
  }
  while (RgtEncdr.getRawPosition() <= 0) {
    RgtMtr.writeMicroseconds(1650);
  }
  RgtMtr.writeMicroseconds(1500);
  delay(500);
  Wrist.write(120);
  delay(500);
  AnyUse = (LftEncdr.getRawPosition() + 35u+
  0);
  while (LftEncdr.getRawPosition() < AnyUse) {
    LftMtr.writeMicroseconds(1800);
    RgtMtr.writeMicroseconds(1550);
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  delay(1000);
  LftEncdr.zero();
  RgtEncdr.zero();
  while (LftEncdr.getRawPosition() < 1700) {
    WriteForwardSpeed(1700);
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  delay(1000);
  DropOff();
}
void DropOff() {//robot under/past overhang, reach up and attach tesseract, then compress and roll back, return to main switch check
  Grip.write(170);
  delay(500);
  ArmBase.write(90);
  delay(300);
  ArmBend.write(40);
  Wrist.write(30);
  delay(3000);
  while (analogRead(2) > 800) { // over 1000 --> light, less than 500 -->dark
    LftMtr.writeMicroseconds(1340);
    RgtMtr.writeMicroseconds(1340);
  }
  delay(150);
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  delay(500);
  Wrist.write(40);
  delay(500);
  Grip.write(110);
  delay(50);
  Wrist.write(20);
  for (int i = 80; i <= 150; i += 5) {
    ArmBend.write(i);
    delay(100);
  }

  LftEncdr.zero();
  while (LftEncdr.getRawPosition() < 90) {
    WriteForwardSpeed(1700);
  }

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);
  for (int i = 85; i >= 45; i -= 5) {
    ArmBase.write(i);
    delay(100);
  }
  Wrist.write(110);
  delay(1000);
  AnyUse = (RgtEncdr.getRawPosition() - 480);
  while (RgtEncdr.getRawPosition() > AnyUse) {
    RgtMtr.writeMicroseconds(1350);
    LftMtr.writeMicroseconds(1350);
  }
  RgtMtr.writeMicroseconds(1500);
  LftMtr.writeMicroseconds(1500);
  pickedUp++;
  AnyUse = (RgtEncdr.getRawPosition() - 480);
  while (RgtEncdr.getRawPosition() > AnyUse) {
    RgtMtr.writeMicroseconds(1350);
  }
  RgtMtr.writeMicroseconds(1500);
  delay(1000);
  return;
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
    mtrPID.SetMode(MANUAL);
    delayMicroseconds(20);
    mtrPID.SetMode(AUTOMATIC);
    PIDSpeed(accSpd);//sends the current speed for PID control to right motor
  }
  mtrPID.SetSampleTime(10);//sets sampling time for PID control
  PIDSpeed(uSSpd);//sets first datapoint for target speed
  //Serial.println("Finished accelerating");
}
void PIDSpeed(unsigned uSSpd) { //used to ensure robot travels straight during constant velocity
  mtrPID.SetTunings(Kp, Ki, Kd);
  PIDLft = LftEncdr.getSpeed();//set point
  PIDRgt = RgtEncdr.getSpeed();//monitored variable

  mtrPID.Compute();//computes using Kp, Ki, Kd

  LftMtr.writeMicroseconds(uSSpd);//writes desired pwm pulse to left motor
  RgtMtr.writeMicroseconds(PIDRgtPwr);//writes controled pwm pulse to right motor
}
void DebuggerModule() {
  //Debugger module -> all debugger code can go here

  #ifdef DEBUG_HALL_SENSOR
  Serial.println((analogRead(HallLft) - NOFIELDLFT) * TOMILLIGAUSS / 1000);
  Serial.println((analogRead(HallRgt) - NOFIELDRGT) * TOMILLIGAUSS / 1000);
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
