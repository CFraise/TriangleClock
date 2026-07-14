#include "Motor.h"
#include "WiFiOTA.h"
#include "LEDDisplay.h"
#include <Adafruit_Sensor.h>
#include <ArduinoOTA.h>

extern int hours_now;

#define RPM_START   20

Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x42);
Adafruit_StepperMotor *myMotor = AFMS.getStepper(NUMOFMOTORSTEPS, 1);

int orientationStepCounter = 0;
int orientationDirectionMove = FORWARD;
int orientationNow = MIDDAY;

// Sets up the ADXL343BCCZ which sits on the triangleClock LED board. Set to 2G since we are only expecting gravity as a force
// Return: 1 if connection succeeded or -1 if it failed
int setupAccel()
{
  logPrintln("Accelerometer Test");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    logPrintln("FAIL no ADXL343 found");
    return -1;
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_2_G);
  return 1;
}

// Initialize the motor based on the adafruit shiel code
// Return: 1 if connection succeeded or -1 if it failed
int setupMotor()
{
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    logPrintln("Could not find Motor Shield. Check wiring.");
    return -1;
  }
  logPrintln("Motor Shield found.");
  myMotor->setSpeed(RPM_START);  // 10 rpm

  return 1;
}

#define HISPEED     500
#define MEDSPEED    200
#define SLOWSPEED   100
#define STARTSPEED  200


//Uses the accelerometer and motors to position the clock upright, flashing the lights to indicate the direction it is going
void pointAccelDown()
{
  bool finished = false;
  int moveStepperFlag = 0;
  int checkAccelFlag = 1;
  int stateMachine = 0;

  int direction = FORWARD;
  int moveType = DOUBLE;
  int numOfReads = 3;
  int speed = 1;
  float accelRead = 0; //Latest accelerometer reading
  float prevAccelRead = 0; //Previous Accelerometer reading
  float prev2AccelRead = 0; //The reading before the previous accelerometer reading

  float calcBright = 0;
  unsigned long int startMillis = millis();

  while(!finished && millis() <= (startMillis+40000))
  {
    if(checkAccelFlag && moveStepperFlag == 0)
    {
      delay(1000);
      accelRead = readAccel(numOfReads);
      if(accelRead < 5)       speed = HISPEED;
      else if(accelRead < 9)  speed = MEDSPEED;
      else                    speed = SLOWSPEED;

      switch(stateMachine)
      {
        case 0: // Read accel and try moving forward
          logPrint("State 0 - Try moving somewhere\nStarting Acceleration: ");
          logPrintln(accelRead);
          prevAccelRead = accelRead;
          moveStepperFlag = STARTSPEED;
          checkAccelFlag = 0;
          stateMachine++;
          break;
        case 1: // Finished test direction, see if it was a good choice
          logPrint("State 1 - Finished test move, was the direction correct?\nAcceleration: ");
          logPrintln(accelRead);
          if(accelRead > prevAccelRead) //Going the right way
          {
            logPrintln("It was correct, continuing in this direction...");
            stateMachine = 3;
            moveStepperFlag = speed;
            checkAccelFlag = 0;
          } else {
            logPrintln("Accel went down, trying the other direction...");
            stateMachine = 2;
            direction = BACKWARD;
            moveStepperFlag = STARTSPEED;
            checkAccelFlag = 2;
          }
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 2:
          logPrintln("State 2 - Test that reversing direction increases accel value (otherwise board is horizontal or pointing down already and we are done");
          if(checkAccelFlag == 2)
          {
            logPrint("1st read: ");
            logPrintln(accelRead);
            moveStepperFlag = STARTSPEED;
          } else {
            logPrint("2nd read: ");
            logPrintln(accelRead);
            if(accelRead > prevAccelRead && prevAccelRead > prev2AccelRead)
            {
              logPrintln("Now moving in the correction direction, moving to state 3...");
              stateMachine = 3;
              moveStepperFlag = speed;
            } else if(prevAccelRead > accelRead && prevAccelRead > prev2AccelRead) {
              logPrintln("Starting spot was pointing down, entering final tuning state 4...");
              stateMachine = 4;
              direction = FORWARD;
              moveStepperFlag = 1;
            } else {
              logPrintln("accel reads don't make sense, the board is probably horizontal or something, exiting this function...");
              finished = true;
            }
          }
          checkAccelFlag = 0;
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 3:
          if(accelRead > prevAccelRead)
          {
            logPrint("State 3 - Keep on moving...\nAccel: ");
            logPrintln(accelRead);
            moveStepperFlag = speed;
          } else {
            logPrint("State 3 - Moved past down, time to fine tune...\nAccel: ");
            logPrintln(accelRead);
            // Reverse direction
            if(direction == FORWARD)  direction = BACKWARD;
            else                      direction = FORWARD;
            moveStepperFlag = 1;
            stateMachine = 4;
            moveType = MICROSTEP;
            numOfReads = 50;
          }
          checkAccelFlag = 0;
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 4:
          logPrint("State 4 - Move 1 step at a time until you've found the minimum value...\nAccel: ");
          logPrint(accelRead);
          if(accelRead > prevAccelRead)
          { //keep inching
            moveStepperFlag = 1;
          } else {
            logPrintln("Previous direction was the max acceleration, move back there and finish...");
            // Reverse direction
            if(direction == FORWARD)  direction = BACKWARD;
            else                      direction = FORWARD;
            moveStepperFlag = 1;
            stateMachine = 5;
          }
          break;
        case 5:
          logPrint("Youre done!\nAccel: ");
          logPrint(accelRead);
          myMotor->release();
          finished = true;
          break;
      }
    }

    if(moveStepperFlag > 0) {
      myMotor->onestep(direction, moveType);
      moveStepperFlag--;
      if(moveStepperFlag == 0 && checkAccelFlag != 2)
        checkAccelFlag = 1;
    }


    for(int i=0; i < 7; i++)
    {
      calcBright = beatsin8(20,0,255, 0, -13*i)*max(1.0, ((i+4)/9.0));
      if(i == 6) {
        calcBright += 140;
        if( calcBright > 255)
          calcBright = 255;

      }
      leds[7+i] = CHSV( 0 ,255,(int) calcBright);
      leds[20-i] = leds[7+i];
    }

    FastLED.show();
    yield();
    ArduinoOTA.handle();
  }
  orientationNow = 0;
}

// readAccel will read the accelerometer x amount of times and average the result of the y value
// input: numOfReads is an int saying how many reads to average across
// output: the averaged value in m/s^2 (negative of it so that gravity is higher)
float readAccel(int numOfReads)
{
  sensors_event_t event;
  accel.getEvent(&event);

  float accelValueY = 0;

  for(int i=0; i<numOfReads; i++)
  {
    accel.getEvent(&event);
    accelValueY += event.acceleration.y;
  }
  accelValueY /= numOfReads;

  return accelValueY;
}

// Runs after the accel is pointing down, so it has a splashy animation while it cruises to the actual first posision now that the time is known
void initialMove()
{
  if(hours_now >= 3 && hours_now < 11) { //morning
    myMotor->step(NUMOFMOTORSTEPS, FORWARD, DOUBLE);
    orientationNow = MORNING;

  } else if( hours_now >= 11 && hours_now < 19) { //midday
    orientationNow = MIDDAY;
  } else { //night
    myMotor->step(NUMOFMOTORSTEPS, BACKWARD, DOUBLE);
    orientationNow = NIGHT;
  }
}
