#define NOFIELD 505L
#define TOMILLIGAUSS 976L//AT1324: 5mV = 1 Gauss, 1024 analog steps to 5V  
const unsigned HallSensor1 = A0;
const unsigned HallSensor2 = A1;
unsigned long sensorValue = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println((analogRead(HallSensor1) - NOFIELD) * TOMILLIGAUSS / 1000);
}

/*
NOTES:
AVERAGE IDLE VALUE: 10-12
*/
