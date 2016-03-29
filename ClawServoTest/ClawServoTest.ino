#include <Servo.h>

Servo Grip;
Servo Wrist;
int GripPin = 9;
int WristPin = 11;
long sweepVal = 0;
void setup() {
  // put your setup code here, to run once:
  Grip.attach(GripPin);
  Wrist.attach(WristPin);

}

void loop() {
  // put your main code here, to run repeatedly:
  sweepVal++;
  Grip.write((sweepVal%180));
  Wrist.write((sweepVal%180));
  delay(10);
}
