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

#define STEPS_PER_REV           1600
#define RIGHT                   1
#define LEFT                    -1
#define A                       1
#define B                       -1

//PIN definitions
#define PIN_STEP                12
#define PIN_DIRECTION           11
#define PIN_LED                 13
#define PIN_ROTATE_LEFT         7
#define PIN_ROTATE_RIGHT        6
#define PIN_SAVE_POSITION_A     5
#define PIN_SAVE_POSITION_B     4
#define PIN_GOTO_POSITION_A     3
#define PIN_GOTO_POSITION_B     2
#define PIN_MAX_SPEED           0
#define PIN_ACCELERATION        1
// These are the constants that define the speed associated with the MaxSpeed pot
#define MAX_STEPS_PER_SECOND    1000    // At 200 s/r and 1/8th microstepping, this will be 333 rev/minute
#define MIN_STEPS_PER_SECOND    27      // At 200 steps/rev and 1/8th microstepping, this will be 1 rev/minute
// Change this value to scale the acceleration pot's scaling factor
#define ACCEL_RATIO             1


//global variables
int accel = 0;
float fStepsPerSecond = 0.0;
long savedPosA = 0;
long savedPosB = 0;
unsigned int loopCtr = 0;
AccelStepper stepper(1, 12, 11);

void readAccel(){
  accel = analogRead(PIN_ACCELERATION)/ACCEL_RATIO;
}

void readStepsPerSecond(){
  // Grab both speed and accel before we start
  int maxSpeed = analogRead(PIN_MAX_SPEED);

  // Do the math to scale the 0-1023 value (maxSpeed) to
  // a range of MIN_STEPS_PER_SECOND to MAX_STEPS_PER_SECOND
  fStepsPerSecond = max(MIN_STEPS_PER_SECOND + (maxSpeed / 1023.0) * (MAX_STEPS_PER_SECOND - MIN_STEPS_PER_SECOND), 1000);
}

void readAccelAndStepsOneInAWhile(){
    // Only read these once in a while because they take a LONG time
    if(loopCtr %100 == 0) readStepsPerSecond();

    // Read in the acceleration analog value
    // This needs to be scaled too, but to what?
    if(loopCtr %200 == 0) readAccel();

    loopCtr++;
}

boolean shouldRotate(int direction){
    int pin = direction == 1 ? PIN_ROTATE_RIGHT : PIN_ROTATE_LEFT; 
    return digitalRead(pin);
}

boolean shouldSavePosition(int position){
    int pin = position == 1 ? PIN_SAVE_POSITION_A : PIN_SAVE_POSITION_B; 
    return digitalRead(pin);
}

boolean shouldGotoPosition(int position){
    int pin = position == 1 ? PIN_GOTO_POSITION_A : PIN_GOTO_POSITION_B; 
    return digitalRead(pin);
}

void setDirection(int direction){
    stepper.setSpeed(direction * fStepsPerSecond);
}

boolean rotateIfNeeded(int direction){
  if(shouldRotate(direction))  {
    setDirection(direction);
    
    while(shouldRotate(direction)){
      CheckPots();
      stepper.runSpeed();
      setDirection(direction);
    }

    return true;
  }

  return false;
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

boolean gotoPositionIfNeeded(int position){
  if (shouldGotoPosition(position))  {
    stepper.setAcceleration(0);
    stepper.runToNewPosition(stepper.currentPosition());
    stepper.setMaxSpeed(fStepsPerSecond);
    stepper.setAcceleration(accel);
    stepper.runToNewPosition(position == A ? savedPosA : savedPosB);

    //"waste" additional signals from the button
    while(shouldGotoPosition(position));

    return true;
  }

  return false;
}

void setup() {
  pinMode(PIN_STEP,             OUTPUT);
  pinMode(PIN_DIRECTION,        OUTPUT);
  pinMode(PIN_LED,              OUTPUT);
  pinMode(PIN_ROTATE_LEFT,      INPUT);
  pinMode(PIN_ROTATE_RIGHT,     INPUT);
  pinMode(PIN_SAVE_POSITION_A,  INPUT);
  pinMode(PIN_SAVE_POSITION_B,  INPUT);
  pinMode(PIN_GOTO_POSITION_A,  INPUT);
  pinMode(PIN_GOTO_POSITION_B,  INPUT);

  stepper.setMaxSpeed(800.0);
  stepper.setAcceleration(600.0);

  // Grab both speed and accel before we start
  readAccelAndStepsOneInAWhile();
}

void loop() {
  // First, we need to see if either rotate button is down. They always take precedence.
  boolean rotatingRight = rotateIfNeeded(RIGHT);
  if(!rotatingRight) rotateIfNeeded(LEFT);
  
  // Go see if we need to update our analog conversions
  readAccelAndStepsOneInAWhile();
  
  savePositionIfNeeded(A);
  savePositionIfNeeded(B);
  
  boolean goingToA = gotoPositionIfNeeded(A);
  if(!goingToA) gotoPositionIfNeeded(B);
}