/*
 Stepper Motor Single Channel Manual Controller
 language: Wiring/Arduino

 This program drives a single bi-polar stepper motor using an EasyDriver
 stepper motor controller from SparkFun.
 It takes input from a set of switches and potentiometers.
 
 The goal is to create smooth motion of the steppper motor without having a 
 PC attached to the Arduino.

 The motor moves 1600 steps in one rotation. (8th step microstepping on a 200 s/r motor)

 Created 08/18/2010
 by Brian Schmalz
 
 Version 0.2  01/30/2011  Added accel/decl
 Version 0.3  10/09/2011  Fixed two bugs found by Daniel Schweinert:
                           1) Speed Pot not working during Rotate Left or Rotate Right moves
                           2) When Goto A or Goto B is pressed after using Rotate Left or Right, motor moves in the
                           wrong directly for a little bit before moving in the right direction.
*/

#include <AccelStepper.h>

// Define some steppers and the pins the will use
AccelStepper stepper(1, 12, 11); 

#define stepsPerRev      1600
#define RIGHT = 1;
#define LEFT = -1;
#define A = 1;
#define B = -1;

//PIN definitions
#define PIN_STEP            12
#define PIN_DIRECTION       11
#define PIN_LED             13
#define PIN_ROTATE_LEFT     7
#define PIN_ROTATE_RIGHT    6
#define PIN_SAVE_POSITION_A 5
#define PIN_SAVE_POSITION_B 4
#define PIN_GOTO_POSITION_A 3
#define PIN_GOTO_POSITION_B 2
#define PIN_MAX_SPEED       0
#define PIN_ACCELERATION    1

// Set this to zero if you don't want debug messages printed
#define printDebug       0

// These are the constants that define the speed associated with the MaxSpeed pot
#define MAX_STEPS_PER_SECOND  1000    // At 200 s/r and 1/8th microstepping, this will be 333 rev/minute
#define MIN_STEPS_PER_SECOND  27      // At 200 steps/rev and 1/8th microstepping, this will be 1 rev/minute

// Change this value to scale the acceleration pot's scaling factor
#define ACCEL_RATIO           1

int buttonState = 0;
int stepNumber = 0;
int curSpeed = 100;
int dir = 0;
int maxSpeed = 0;
int accel = 0;
long savedPosA = 0;
long savedPosB = 0;

int loopCtr = 0;
  
float fMaxSpeed = 0.0;
float fStepsPerSecond = 0.0;

void setup() 
{
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIRECTION, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ROTATE_LEFT, INPUT);
  pinMode(PIN_ROTATE_RIGHT, INPUT);
  pinMode(PIN_SAVE_POSITION_A, INPUT);
  pinMode(PIN_SAVE_POSITION_B, INPUT);
  pinMode(PIN_GOTO_POSITION_A, INPUT);
  pinMode(PIN_GOTO_POSITION_B, INPUT);
  
  if (printDebug)
  {
    // Initialize the Serial port
    Serial.begin(9600);
  }
  
  // blink the LED:
  blink(2);

  stepper.setMaxSpeed(800.0);
  stepper.setAcceleration(600.0);

  // Grab both speed and accel before we start
  maxSpeed = analogRead(PIN_MAX_SPEED);
  // Do the math to scale the 0-1023 value (maxSpeed) to 
  // a range of MIN_STEPS_PER_SECOND to MAX_STEPS_PER_SECOND
  fMaxSpeed = maxSpeed / 1023.0;
  fStepsPerSecond = MIN_STEPS_PER_SECOND + (fMaxSpeed * (MAX_STEPS_PER_SECOND - MIN_STEPS_PER_SECOND));
  if (fStepsPerSecond > 1000)
  {
    fStepsPerSecond = 1000;
  }
  accel = analogRead(PIN_ACCELERATION)/ACCEL_RATIO;
}

//TODO: is bool the right type?
bool shouldRotate(int direction){
    int pin = direction == 1 ? PIN_ROTATE_RIGHT : PIN_ROTATE_LEFT; 
    return digitalRead(pin);
}

bool shouldSavePosition(int position){
    int pin = position == 1 ? PIN_SAVE_POSITION_A : PIN_SAVE_POSITION_B; 
    return digitalRead(pin);
}

bool shouldGotoPosition(int position){
    int pin = position == 1 ? PIN_GOTO_POSITION_A : PIN_GOTO_POSITION_B; 
    return digitalRead(pin);
}

void setDirection(int direction){
    stepper.setSpeed(direction * fStepsPerSecond);
}

void rotateIfNeeded(int direction){
  if(shouldRotate(direction))  {
    setDirection(direction);
    
    while(shouldRotate(direction)){
      CheckPots();
      stepper.runSpeed();
      setDirection(direction);
    }
  }
}

void savePositionIfNeeded(int position){
  if(shouldSavePosition(position)){
    
    //keep the current position
    if(position == A) savedPosA = stepper.currentPosition();
    else if(position == B) savedPosB = stepper.currentPosition();
    
    //"waste" additional signals from the button
    while(shouldSavePosition(position));
  }
}

void gotoPositionIfNeeded(int position){
  if (shouldGotoPosition(position))  {
    stepper.setAcceleration(0);
    stepper.runToNewPosition(stepper.currentPosition());
    stepper.setMaxSpeed(fStepsPerSecond);
    stepper.setAcceleration(accel);
    stepper.runToNewPosition(position == A ? savedPosA : savedPosB);

    //"waste" additional signals from the button
    while(shouldGotoPosition(position));
  }
}

void loop() {  
  // First, we need to see if either rotate button is down. They always take precedence.
  rotateIfNeeded(RIGHT);
  rotateIfNeeded(LEFT);
  
  // Go see if we need to update our analog conversions
  CheckPots();
  
  savePositionIfNeeded(A);
  savePositionIfNeeded(B);
  
  gotoPositionIfNeeded(A);
  gotoPositionIfNeeded(B);
}

// Blink the reset LED:
void blink(int howManyTimes) 
{
  int i;
  for (i=0; i < howManyTimes; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(200);
    digitalWrite(PIN_LED, LOW);
    delay(200);
  }
}

void CheckPots(void)
{
  loopCtr++;

  // Only read these once in a while because they take a LONG time
  if (loopCtr == 100)
  {
    maxSpeed = analogRead(PIN_MAX_SPEED);

    // Do the math to scale the 0-1023 value (maxSpeed) to 
    // a range of MIN_STEPS_PER_SECOND to MAX_STEPS_PER_SECOND
    fMaxSpeed = maxSpeed / 1023.0;
    fStepsPerSecond = MIN_STEPS_PER_SECOND + (fMaxSpeed * (MAX_STEPS_PER_SECOND - MIN_STEPS_PER_SECOND));
    if (fStepsPerSecond > 1000)
    {
      fStepsPerSecond = 1000;
    }
  }

  // Read in the acceleration analog value
  // This needs to be scaled too, but to what?
  if (loopCtr >= 200)
  {
    accel = analogRead(PIN_ACCELERATION)/ACCEL_RATIO;
    loopCtr = 0;
  } 
}
