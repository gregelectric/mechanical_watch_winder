
/*
 Stepper Motor Control - speed control

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.
 A potentiometer is connected to analog input 0.

 The motor will rotate in a clockwise direction. The higher the potentiometer value,
 the faster the motor speed. Because setSpeed() sets the delay between steps,
 you may notice the motor is less responsive to changes in the sensor value at
 low speeds.

 Created 30 Nov. 2009
 Modified 28 Oct 2010
 by Tom Igoe

 */

#include <Stepper.h>

const int stepsPerRevolution = 100;  // change this to fit the number of steps per revolution
// for your motor


// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 4, 6, 5, 7);

int stepCount = 0;  // number of steps the motor has taken
int motorSpeed = 200;
int num_steps = 1500;

void setup() 
{
  myStepper.setSpeed(motorSpeed);
}

void loop() 
{
  // Clockwise
  for(int i = 0; i < num_steps; i++)
  {
    // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 100);
  }

  // Counter Clockwise
  for(int i = 0; i < num_steps; i++)
  {
    // step 1/100 of a revolution:
    myStepper.step(-stepsPerRevolution / 100);
  }
}
