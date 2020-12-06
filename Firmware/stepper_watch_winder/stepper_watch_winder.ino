
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

#define motor_pin_1 4
#define motor_pin_2 5
#define motor_pin_3 6
#define motor_pin_4 7

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);

int stepCount = 0;  // number of steps the motor has taken
int motorSpeed = 100;
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
  
  // setup the pins on the microcontroller:
  pinMode(motor_pin_1, INPUT);
  pinMode(motor_pin_2, INPUT);
  pinMode(motor_pin_3, INPUT);
  pinMode(motor_pin_4, INPUT);
  delay(60000);
  pinMode(motor_pin_1, OUTPUT);
  pinMode(motor_pin_2, OUTPUT);
  pinMode(motor_pin_3, OUTPUT);
  pinMode(motor_pin_4, OUTPUT);
    
  // Counter Clockwise
  for(int i = 0; i < num_steps; i++)
  {
    // step 1/100 of a revolution:
    myStepper.step(-stepsPerRevolution / 100);
  }

   // setup the pins on the microcontroller:
  pinMode(motor_pin_1, INPUT);
  pinMode(motor_pin_2, INPUT);
  pinMode(motor_pin_3, INPUT);
  pinMode(motor_pin_4, INPUT);
  delay(60000);
  pinMode(motor_pin_1, OUTPUT);
  pinMode(motor_pin_2, OUTPUT);
  pinMode(motor_pin_3, OUTPUT);
  pinMode(motor_pin_4, OUTPUT);
}
