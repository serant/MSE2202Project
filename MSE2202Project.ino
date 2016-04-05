/*
RELEASE 1.0
Project code used in MSE2202B Design Showcase
*/

#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <uSTimer2.h>
#include <PID_v1.h>

//==DEBUGGERS: UNCOMMENT TO DEBUG==//
//#define DEBUG_HALL_SENSOR
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER
//#define DEBUG_ENCODERS
//#define DEBUG_TRACKING
//#define DEBUG_PID

//===========PIN OUTS=============//
const int UltrasonicPing = 2;
//ULTRASONIC DATA RETURN IN D3
const int RgtMtrPin = 4;
const int LftMtrPin = 5;
const int ArmBasePin = 6;
const int ArmBendPin = 7;
const int GripPin = 10;
const int WristPin = 11;
const int HallRgt = A0;
const int HallLft = A1;
const int GripLight = A2;
const int HallGrip = A3;
const int ci_I2C_SDA = A4; // I2C data = white -> Nothing will be plugged into this
const int ci_I2C_SCL = A5; // I2C clock = yellow -> Nothing will be plugged into this

//===========DATA VARIABLES=======//

//-----sensor data----//

//Hall Sensor Data
//right and left hall sensors on front are sampled at beginning to determine idle value
int NOFIELDRGT = 0;
int NOFIELDLFT = 0;
#define NOFIELDGRIP 510L//grip idle value is consistently 510 L
#define TOMILLIGAUSS 976L//Convert to milli Gauss forAT1324: 5mV = 1 Gauss, 1024 analog steps to 5V 
int currentHallRead; //used to compare hall values for grip sensor
int lastHallRead; //used to compare hall values for grip sensor
unsigned HallThreshold = 6; //threshold for hall sensor to determin tesseract is within its vicinity

//Line Tracker Data
unsigned GripLightData = 0; //holds value for grip line tracker data

//Ultrasonic Data
unsigned long UltrasonicDistance = 0;

//-------PID Control-------//
double PIDRgt, PIDRgtPwr, PIDLft;//monitored value, controlled value, setpoint
double Kp = 11.9, Ki = 100, Kd = 0.00001; //PID parameters
unsigned accSpd = 0;//used for acceleration of the robot to allow PID to operate properly
PID mtrPID(&PIDRgt, &PIDRgtPwr, &PIDLft, Kp, Ki, Kd, DIRECT);//PID control to allow robot to drive straight
unsigned long PIDTimer;

//-------tracking using encoders-----------/
const double CE = 637;//pulses per revolution
const double CF = ((3.14159 * 69.85) / CE); //Conversion factor, traslates encoder pulses to linear displacement
double DelLft = 0;
double DelRgt = 0;
double DelDsp = 0;
long TotalDsp = 0;
double SvdDsp = 0;
double Dsp = 0;
double OrTheta = 0;
double dTheta = 0;
double PolTheta = 0;
double FindTheta = 0;
double PickUpTheta = 0;
double XPstn = 0;
double YPstn = 0;
bool Light = false;
int Line = 0;
double SvdDelDisp = 0;
unsigned targetTheta = 0; //used for reorienting robot

//-----mechanics data----//
//Speeds
unsigned WheelPerimeter = (69.85 * PI) / 10; //perimeter of wheel in cm
unsigned Stop = 1500; //speed of robot to stop
unsigned int MotorSpeed;
unsigned int LftMotorSpeed;
unsigned int RgtMotorSpeed;
unsigned long LeftMotorOffset; //used to offset speeds without using PID
unsigned long RightMotorOffset; //used to offset speed without using PID

//Encoder Data
I2CEncoder LftEncdr;
I2CEncoder RgtEncdr;
I2CEncoder ArmBaseEncdr;
I2CEncoder ArmBendEncdr;
long lftEncoderCounter;
long rgtEncoderCounter;
unsigned int LftMotorPos;
unsigned int RgtMotorPos;
unsigned tempEncoderPosition = 0;
long RawLftPrv = 0;
long RawRgtPrv = 0;
double savedLftEncdr = 0;
double savedRgtEncdr = 0;
double LftEncdrCount = 0;
double RgtEncdrCount = 0;
double savedLftEncdrReturn = 0;
double SavedRgtEncdrReturn = 0;

//Servo Data
Servo LftMtr;
Servo ArmBend;    //out -> folded 0->180
Servo ArmBase;    //folded->out  37-180
Servo RgtMtr;
Servo Grip;       //170 closed, 100 open
Servo Wrist;      //0 min folded up, 50 straight out, 180 folded down

//----program flow control//
//Timer
unsigned int timer;
unsigned long timerStart;
unsigned timeRun = 5;


//Flags/Switches
bool TurnRight = true;
bool startTask = true;
unsigned pickedUp;
bool Start = true;

 //Mode Selector Variable
 unsigned int ModeIndex = 4;


 void setup() {
  Serial.begin(9600);//begin serial monitor at 9600 baud
  Wire.begin();
  delay(2000);

  //set up right servo motor
  pinMode(RgtMtrPin, OUTPUT);
  RgtMtr.attach(RgtMtrPin);

  //set up left servo motor
  pinMode(LftMtrPin, OUTPUT);
  LftMtr.attach(LftMtrPin);

  //set up servo motor to control grip
  pinMode(GripPin, OUTPUT);
  Grip.attach(GripPin);
  Grip.write(110); //open grip slightly 

  //set up servo motor to control wrist
  pinMode(WristPin, OUTPUT);
  Wrist.attach(WristPin);
  Wrist.write(80);//open wrist slightly

  // set up encoders right encoder must be set up first since the left is daisy chained
  RgtEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  RgtEncdr.setReversed(false);  // adjust for positive count when moving forward

  LftEncdr.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  LftEncdr.setReversed(true);  // adjust for positive count when moving forward

  //set up arm base servo motor
  pinMode(ArmBasePin, OUTPUT);
  ArmBase.attach(ArmBasePin);
  ArmBase.write(40);

  //set up arm bend/elbow servo motor
  pinMode(ArmBendPin, OUTPUT);
  ArmBend.attach(ArmBendPin);
  ArmBend.write(150);

  //set up VEX line tracker on grip
  pinMode(GripLight, INPUT);

  //ultrasonic setup
  pinMode(UltrasonicPing, OUTPUT);
  pinMode(UltrasonicPing + 1, INPUT);

  //set up PID control
  mtrPID.SetMode(AUTOMATIC);
  mtrPID.SetOutputLimits(1500, 1900);
  mtrPID.SetSampleTime(10);
  
  //initialize hall effect sensors with idle field value
  for(int i = 0; i < 10; i++){
    //sum cummulative data for hall sensors
    NOFIELDLFT += analogRead(HallLft);
    NOFIELDRGT += analogRead(HallRgt);
    delay(100);
  }
  //average data
  NOFIELDLFT = NOFIELDLFT/10;
  NOFIELDRGT = NOFIELDRGT/10;
}

void loop() {
  DebuggerModule();//prints any serial statements defined by debugger at top
  Position();//updates position of robot

  timer = millis() / 1000; //time in seconds

  if (timer >= 240) {  //4 min time limit
    // robot goes home if time limit is reached
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
    case 0: //robot idles

    break;

    //=====CASE 1 IS THE MAIN LOOP FOR MODE 1=====/
    case 1: //robot is in mode 1: searches for tesseracts

      //Looks for Blocks
      //if tesseract is at left hall sensor, call pickup and pass 1 to indicate left 
      if ((analogRead(HallLft) - NOFIELDLFT) > HallThreshold) PickUp(1); 

      //if tesseract is at right hall sensor, call pickup and pass 0 to indicate right
      else if ((analogRead(HallRgt) - NOFIELDRGT) > HallThreshold) PickUp(0); //

      //Pings to detect if wall is in front
      Ping(UltrasonicPing);

      //Looks for Blocks

      //if tesseract triggers left hall sensor
      if (((analogRead(HallLft) - NOFIELDLFT) > 10) ||((analogRead(HallLft) - NOFIELDLFT) < -10)) {//if tesseract is at left hall sensor, call pickup and pass 1 to indicate left
        Serial.println("entered left");
        PickUp(1);
      }

      //if tesseract triggers right hall sensor
      else if (((analogRead(HallRgt) - NOFIELDRGT) > 10) || ((analogRead(HallRgt) - NOFIELDRGT) < -10 )) {
        PickUp(0);
        Serial.println("Entered right");
      }


    else if ((millis() - timerStart) > timeRun) {
      //ultrasonic is more reliable when robot stops: may be due to wiring issue
      LftMtr.writeMicroseconds(1500);//robot stops to ping ultrasonic
      RgtMtr.writeMicroseconds(1500);
      mtrPID.SetMode(MANUAL);//PID stops sampling to prevent negative feedback
      delay(500);//robot delays
      Ping(UltrasonicPing);//robot pings ultrasonic
      mtrPID.SetMode(AUTOMATIC);//PID resumes
        
        //frequency of ultrasonic pings are dependant on how far away the wall is
        if (UltrasonicDistance < 300 && UltrasonicDistance != 0) timeRun = 4000;
        if (UltrasonicDistance < 80 && UltrasonicDistance != 0) timeRun = 1500;
        if (UltrasonicDistance < 50 && UltrasonicDistance != 0) timeRun = 500;
        
        if (UltrasonicDistance < 25 && UltrasonicDistance > 2) {//if robot is close to wall
          RgtMtr.writeMicroseconds(Stop);//stops to prepare for turn
          LftMtr.writeMicroseconds(Stop);
          delay(100);
          if (TurnRight) {//if turning right...
            tempEncoderPosition = LftEncdr.getRawPosition();
            while ((LftEncdr.getRawPosition() < tempEncoderPosition + 960)) {
              LftMtr.writeMicroseconds(1700);//robot turns right
            }

            LftMtr.writeMicroseconds(1500);
            RgtMtr.writeMicroseconds(1500);
          }
          else {//if turning left...
            tempEncoderPosition = RgtEncdr.getRawPosition();
            while (RgtEncdr.getRawPosition() < (tempEncoderPosition + 960)) {
              RgtMtr.writeMicroseconds(1700);//robot turns left
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
        WriteForwardSpeed(1800);//robot moves forward using PID
    }
    break;

    //======CASE 2 IS THE MAIN LOOP FOR MODE 2=====/
    case 2:  
    Serial.println("In mode 2");

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

    while (UltrasonicDistance > 21 || UltrasonicDistance < 10) {//if robot is not in proper position
      for (int i = 0; i < 4; i++) {//ping several times to sample distance
        Ping(UltrasonicPing);
      }
      //move forward a bit
      LftMtr.writeMicroseconds(1650);
      RgtMtr.writeMicroseconds(1660);
      delay(100);
      //stop
      LftMtr.writeMicroseconds(1500);
      RgtMtr.writeMicroseconds(1500);
      delay(100);
    }
    //zero encoders to begin tracking position
    LftEncdr.zero();
    RgtEncdr.zero();

    //turn to the right a bit
    while (LftEncdr.getRawPosition() < 200) {
      LftMtr.writeMicroseconds(1700);
      delay(100);
      LftMtr.writeMicroseconds(1500);
      delay(100);

      //determines if tesseract is within vicinity
      currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
      if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -122)) {
        Move();//calls move to position to pick up tesseract 
      }
    }
    delay(400);
    while (LftEncdr.getRawPosition() > 0) {
      //turn to the left a bit
      LftMtr.writeMicroseconds(1330);
      delay(100);
      LftMtr.writeMicroseconds(1500);
      delay(100);

      //determines if tesseract is within vicinity
      currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
      if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
        Move();//calls move to position to pick up tesseract
      }
    }
    delay(400);
    //moves back to center robot to scan again
    while (RgtEncdr.getRawPosition() < 200) {
      RgtMtr.writeMicroseconds(1630);
      delay(100);
      RgtMtr.writeMicroseconds(1500);
      delay(100);

      //determines if tesseract is within vicinity
      currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
      if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
        Move();//calls move to position to pick up tesseract
      }
    }
    delay(400);

    //moves back to center robot to scan again
    while (RgtEncdr.getRawPosition() > 0) {
      RgtMtr.writeMicroseconds(1370);
      delay(100);
      RgtMtr.writeMicroseconds(1500);
      delay(100);

      //determines if tesseract is within vicinity
      currentHallRead = analogRead(HallGrip); // Hall Grip Values: 515 --> no magnetic field, below 500 --> magnetic field
      if ((currentHallRead - lastHallRead > 12) || (currentHallRead - lastHallRead < -12)) {
        Move();//calls move to position to pick up tesseract
      }
    }
    delay(400);
    break;
  }
}

//========SENSOR FUNCTIONS========//
//Sends out an ultrasonic pulse to be read
void Ping(int x) {
  //Ping Ultrasonic
  digitalWrite(x, HIGH);//x is used since receive module will always be on consecutive digital pin
  mtrPID.SetMode(MANUAL);
  delayMicroseconds(10);//delay for 10 microseconds while pulse is in high
  mtrPID.SetMode(AUTOMATIC);
  digitalWrite(x, LOW); //turns off the signal
  UltrasonicDistance = (pulseIn(x + 1, HIGH, 10000) * 1.1 / 58);//returns in cm
}

//Gathers data from line tracker
void ReadLineTracker() {
  GripLightData = analogRead(GripLight);
}

//--------PID functions---------//
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
}
void PIDSpeed(unsigned uSSpd) { //used to ensure robot travels straight during constant velocity
  mtrPID.SetTunings(Kp, Ki, Kd);
  PIDLft = LftEncdr.getSpeed();//set point
  PIDRgt = RgtEncdr.getSpeed();//monitored variable

  mtrPID.Compute();//computes using Kp, Ki, Kd

  LftMtr.writeMicroseconds(uSSpd);//writes desired pwm pulse to left motor
  RgtMtr.writeMicroseconds(PIDRgtPwr);//writes controled pwm pulse to right motor
}


//======MODE 1 FUNCTIONS=====//
//Called to pick up a tesseract: based on which hall sensor detects inductance the robot positions
//itself to pick up the tesseract
void PickUp(int i) {  //left = 1, right = 0
  //robot has deteced tesseract and uses arm to pick it up, after picked up runs 'GoHome'

  switch (i) {
    case 0://if tesseract activated right sensor
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

    case 1://if tesseract activated left  sensor
    RgtMtr.writeMicroseconds(1450);
    LftMtr.writeMicroseconds(1400);
    delay(1000);
    RgtMtr.writeMicroseconds(Stop);
    LftMtr.writeMicroseconds(Stop);
    delay(200);
    RgtMtr.writeMicroseconds(1600);
    LftMtr.writeMicroseconds(1600);
    delay(1000);
    RgtMtr.writeMicroseconds(Stop);
    LftMtr.writeMicroseconds(Stop);
  }

  delay(500);

  //backs up robot a bit
  LftMtr.writeMicroseconds(1350);
  RgtMtr.writeMicroseconds(1350);
  delay(1500);
  LftMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(Stop);

  //---picks up tesseract--//
  Grip.write(100); //opens grip
  Wrist.write(48); //bends wrist up   
  ArmBend.write(130); //extends arm
  delay(1000);
  ArmBase.write(110); //moves arm down
  delay(1000);
  ArmBase.write(125); //slowly moves arm around tesseract
  delay(1000);
  if (i == 0) {
    //if tesseract is to the right
    LftMtr.writeMicroseconds(1620); 
    RgtMtr.writeMicroseconds(1650);//right motor was damaged so despite pwm pulse, robot will turn to the right
    delay(900);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);
  }
  else {
    LftMtr.writeMicroseconds(1650);
    RgtMtr.writeMicroseconds(1650);//robot moves to the left since right motor was damaged
    delay(900);
    LftMtr.writeMicroseconds(Stop);
    RgtMtr.writeMicroseconds(Stop);
  }
  Grip.write(170);//close grip
  delay(1000);
  ArmBase.write(90);//retract arm
  delay(1000);
  int tempTime = millis();
  while((millis()-tempTime) <= 3000){//moves forward
    WriteForwardSpeed(1700);
  }
  LftMtr.writeMicroseconds(Stop);
  RgtMtr.writeMicroseconds(Stop);

 //checks to ensure robot has picked up magnetic tesseract
 if (!((analogRead(HallGrip) - NOFIELDGRIP) > 8 || ((analogRead(HallGrip) - NOFIELDGRIP) < -8))) {
   ArmBend.write(20);
   delay(1000);
   Grip.write(100);//discards tesseract if not magnetic
   delay(500);
   for (int j = 20; j <= 130; j += 5) {
     ArmBend.write(j);
     delay(100);
   }
 }
 else {//if passed inductance test, robot calls GoHome
   Serial.println("got tesseract");
   GoHome();
 }
}

//Determines the polar and cartesian coordinates of the robot based on encoder values
void Position() {
  // PickUpTheta, FindTheta, SvdRgtEncdr, SvdLftEncdr

  // Distance travelled
  DelRgt = (CF * ((RgtEncdr.getRawPosition()))); // Instantaneous Distance traveled by right Wheel
  DelLft = (CF * ((LftEncdr.getRawPosition()))); // Instantaneous Distnace traveled by left wheel
  DelDsp = (DelRgt + DelLft) / 2; //Distance traveled by the centerpoint of the robot, affected by quadrent
  Dsp = Dsp + DelDsp; //Current Displacement
  TotalDsp = (abs(DelRgt) + abs(DelLft)) / 2; //Total distance travelled

  OrTheta = ((DelRgt - DelLft) / 115.5) * (180 / PI); // Change in orientation, taking starting postion as Theta = 0
  OrTheta = (int)OrTheta % 360; //If the magnitude of the orientation is greater than 360

  XPstn = DelDsp * cos((OrTheta * PI) / 180);
  YPstn = DelDsp * sin((OrTheta * PI) / 180);

  PolTheta = (atan(YPstn / XPstn) * (180 / PI)); //The polar angle of the position of the robot
}

//called after the robot picks up a tesseract and sends the robot to the start point
void GoHome() {
  //robot calculates and saves position and returns to base after tesseract picked up, runs 'Look'
  Position();//updates position
  for (int i = 0; i > 0; i++) {//performed once
    SvdDsp = DelDsp;//saves position and angle
    PickUpTheta = PolTheta;
  }

  if (!TurnRight) { //Turn number is even, right turn == true
    targetTheta = PolTheta;
    while (!(OrTheta < (targetTheta + 185) && OrTheta > (targetTheta + 175))) {
      Serial.println("Alinging Bot, even turn...");
      LftMtr.writeMicroseconds(1350);//turns until facing correct direction
      RgtMtr.writeMicroseconds(1650);
      Position();//updates position each cycle
    }
  }
  else { // Turn number is odd
    targetTheta = PolTheta;
    while (!(OrTheta < (PolTheta + 5) && OrTheta > (PolTheta -5))) { 
      Serial.println("Alinging Bot, odd turn...");
      LftMtr.writeMicroseconds(1350);//turns opposite direction until facing correct direction
      RgtMtr.writeMicroseconds(1650);
      Position();
    }
  }

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  Ping(2);

  savedLftEncdr = abs(LftEncdr.getRawPosition());
  savedRgtEncdr = abs(RgtEncdr.getRawPosition());

  //move robot forward until it is at a specified distance from the home position
  while ((UltrasonicDistance > 19) && (UltrasonicDistance != 0)) {
    WriteForwardSpeed(1700);
    Position();
    Ping(2);
  }

  //determines displacement from its previous position to home position to return to later
  LftEncdrCount = abs(LftEncdr.getRawPosition()) - savedLftEncdr;
  RgtEncdrCount = abs(RgtEncdr.getRawPosition()) - savedRgtEncdr;

  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  PlaceTesseract();//places tesseract on the wall
}

//Robot returns to previous point where it picked up last tesseract 
void Return() {
  Position();//updates position
  //robot rotates until facing correct direction using polar angle OrTheta
  while (!(OrTheta < (PickUpTheta + 5) && OrTheta > (PickUpTheta - 5))) { 
    LftMtr.writeMicroseconds(1650);
    RgtMtr.writeMicroseconds(1350);
  }
  LftMtr.writeMicroseconds(1500);
  RgtMtr.writeMicroseconds(1500);

  //stores current encoder position to prepare to return to previous position
  savedLftEncdrReturn = abs(LftEncdr.getRawPosition());

  //moves forward until robot traverses displacement from return point
  while (abs(LftEncdr.getRawPosition()) < (abs(savedLftEncdrReturn) + LftEncdrCount)) {
    WriteForwardSpeed(1700);
    Position();
  }
  
  //updates displacement of robot centerpoint
  SvdDelDisp = ((DelRgt + DelLft) / 2) + sqrt((XPstn * XPstn) + (YPstn * YPstn));
    while ((((DelRgt) + (DelLft)) / 2) < SvdDelDisp) { 
      WriteForwardSpeed(1700);
      Position();
    }
    LftMtr.writeMicroseconds(1500);
    RgtMtr.writeMicroseconds(1500);

  if (!TurnRight) { //Turn number is even
    //orients robot in correct direction
    while (!(OrTheta < 5 && OrTheta > -5)) {
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);

      Position();
    }
  } 
  else { // Turn number is odd
    //orients robot in correct direction
    while (!(OrTheta < 185 && OrTheta > 175)) {
      LftMtr.writeMicroseconds(1350);
      RgtMtr.writeMicroseconds(1650);
      Position();
    }
  }
}

//robot places tesseract on the wall between lines
void PlaceTesseract() {
  // Allign with wall
  while (!((OrTheta < -210) && (OrTheta > -220))) {
    LftMtr.writeMicroseconds(1625);
    RgtMtr.writeMicroseconds(1375);
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


//==========MODE 2 FUNCTIONS============//
//robot continiously checks wall to see if there is a tesseract available, if found runs 'Move'
void Check() {


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
    if (currentHallReading - lastHallReading > 15) {
      return;
    }
  }
  LftMotorSpeed = 1350;
  LftMtr.writeMicroseconds(LftMotorSpeed);
  for (LftEncoderCounter; LftEncoderCounter > 0; LftEncoderCounter--) {
    int currentHallReading = analogRead(HallGrip);
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

//detected tesseract on wall, pick it up, turn, move under beam, then run DropOff
//robot picks up tesseract from wall, drives under beam and hangs tesseract on overhang, returns back under beam, runs 'Check'
void Move() {
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
  tempEncoderPosition = (LftEncdr.getRawPosition() + 35u+
    0);
  while (LftEncdr.getRawPosition() < tempEncoderPosition) {
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

//robot under/past overhang, reach up and attach tesseract, then compress and roll back, return to main switch check
void DropOff() {
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
  tempEncoderPosition = (RgtEncdr.getRawPosition() - 480);
  while (RgtEncdr.getRawPosition() > tempEncoderPosition) {
    RgtMtr.writeMicroseconds(1350);
    LftMtr.writeMicroseconds(1350);
  }
  RgtMtr.writeMicroseconds(1500);
  LftMtr.writeMicroseconds(1500);
  pickedUp++;
  tempEncoderPosition = (RgtEncdr.getRawPosition() - 480);
  while (RgtEncdr.getRawPosition() > tempEncoderPosition) {
    RgtMtr.writeMicroseconds(1350);
  }
  RgtMtr.writeMicroseconds(1500);
  delay(1000);
  return;
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
