#include <Stepper.h>

#define STEPS 200

Stepper stepper(STEPS, 4, 5, 6, 7);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Stepper test!");
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(30);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Forward");
  //stepper.step((STEPS/3*52/22)+2);
  delay(8000);
}
