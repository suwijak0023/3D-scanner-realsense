#include <Stepper.h>
const int stepsPerRevolution = 2048;
const int rpm = 10;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

void setup(){
  myStepper.setSpeed(rpm);
  Serial.begin(9600);
}
void loop(){
  if (Serial.available() > 5) {
    bool negative = Serial.read();
    long steps = 0;
    for (int i = 0; i <= 5; i++){
      steps = (steps * 10) + Serial.read();
    }
    steps = (steps+1)/10;
    if (negative){
      steps = -steps;
    }
    myStepper.step(steps);
    Serial.print(steps);
  }
}
