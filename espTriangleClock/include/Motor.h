#pragma once

#include <Adafruit_MotorShield.h>
#include <Adafruit_ADXL343.h>

#define NUMOFMOTORSTEPS 1024

#define MORNING 0
#define MIDDAY  1
#define NIGHT   2

extern Adafruit_MotorShield AFMS;
extern Adafruit_StepperMotor *myMotor;

extern int orientationNow;
extern int orientationStepCounter;
extern int orientationDirectionMove;

int setupMotor();
int setupAccel();
float readAccel(int numOfReads);
void pointAccelDown();
void initialMove();
