/*****************************************************
*
* Smooth move.
* Only
*
*****************************************************/

#define DIR_PIN 2
#define STEP_PIN 3

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}

void loop(){
  rotate(4800, 0.5);
  delay(2000);

  rotate(-800, 0.8); //reverse
  delay(3000);
}

void setDirection(int steps){
  digitalWrite(DIR_PIN,steps > 0 ? HIGH:LOW);
}

//rotate a specific number of microsteps (8 microsteps per step)
//negitive for reverse movement
//a 200 step stepper would take 1600 micro steps for one full revolution
//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
void rotate(int steps, float speed){
  setDirection(steps);

  float usDelay = (1/speed) * 70;
  for(int i=0; i < abs(steps); i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
}