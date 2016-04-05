//=========DIGITAL============//
const int UltrasonicPing = 2; //front mounted ultrasonic ping
const int UltrasonicData = 3; //front mounted ultrasonic data
const int RgtMtrPin = 4; //right wheel drive motor PWM 
const int LftMtrPin = 5; //left wheel drive motor PWM
const int ArmBasePin = 6; //arm base motor PWM
const int ArmBendPin = 7; //arm bend motor PWM
const int UltrasonicPingSide = 8 //side mounted ultrasonic ping
const int UltrasonicPingSideData = 9; //side mounted ultrasonic data
const int GripPin = 10; //grip MSG90 PWM
const int WristPin = 11; //wrist MSG90 PWM

//=========ANALOG=======/
const int HallRgt = A0; //right front mounted hall sensor input
const int HallLft = A1; //left front mounted hall sensor input
const int GripLight = A2; //line tracker on claw sensor input
const int HallGrip = A3; //hall sensor on claw sensor input
const int ci_I2C_SDA = A4; // I2C data = white -> Nothing will be plugged into this
const int ci_I2C_SCL = A5; // I2C clock = yellow -> Nothing will be plugged into this

