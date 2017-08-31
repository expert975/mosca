/*
  Rodrigo Martins
  My first class - learning C++ and OOP
  PID control
*/

#include <Arduino.h>

class PID
{
public: // --rething data types
  float error; //distance from desired set point to the feedback
  float deltaTime; //time between calculations  --maybe should use unsigned long
  long lastTime; //time at last calculation   --maybe should use unsigned long
  long time; //stores time
  float setpoint; //desired setpoint; goal
  float kP, kI, kD; //PID tuning variables
  float P, I, D; //proportional, integral and derivative terms
  float outMin = 0; //smaller value possible for output --does it get assigned to floating point values?
  float outMax = 255; //larger value possible for output
  float feedbackInput; //input from loop feedback
  float lastFeedbackInput; //last feedback
  float controlOutput; //manipulated variable; output

  PID( float _kP, float _kI, float _kD ) //class constructor
  {
    kP = _kP;
    kI = _kI;
    kD = _kD;

    error = 0;
    deltaTime = 0;
    lastTime = millis();
    time = millis();
    setpoint = 0;

    P = 0;
    I = 0;
    D = 0;

    feedbackInput = 0;
    lastFeedbackInput = 0;
    controlOutput = 0;


  }

  void calculate(float _feedbackInput) //calculates correction
  {
    feedbackInput = _feedbackInput;
    error = setpoint - feedbackInput; //error in the contolled system
    time = millis();
    deltaTime = float(time - lastTime)/1000; //delta time in seconds

    P = kP*error;
    I = I + kI*error*deltaTime;
    D =  kD*(lastFeedbackInput - feedbackInput)/deltaTime;

    // if (I > outMax) I = outMax;
    // if (I < outMin) I = outMin;
    I = constrain(I, outMin, outMax); //prevent reset windup, where I grows more
                                      //than the system can respond
    controlOutput = P + I + D;

    // if (controlOutput > outMax) controlOutput = outMax;
    // if (controlOutput < outMin) controlOutput = outMin;
    controlOutput = constrain(controlOutput, outMin, outMax); //restrict PID output to control output

    lastTime = time;
    lastFeedbackInput = feedbackInput;
    //return controlOutput;
  }


private:

}; //class ends here

PID mosca(0, 0, 0); //start PID object

//Sensor input
int sensorOutR = A0;
int sensorInR = A1;
int sensorCenter = A2;
int sensorInL = A3;
int sensorOutL = A4;

//Engine output
int engineR = 5;
int engineL = 6;

byte speed = 100; //how fast we should go
int controlOutput; //correction value

void setup()
{
  //Sensor input
  pinMode(sensorOutR, INPUT);
  pinMode(sensorInR, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorInL, INPUT);
  pinMode(sensorOutL, INPUT);

  //Engine OUTPUT
  pinMode(engineL, OUTPUT);
  pinMode(engineR, OUTPUT);

  mosca.setpoint = 0;
}

void loop ()
{
  mosca.feedbackInput = sensorInL - sensorInR; //how far from the line center
  controlOutput = mosca.controlOutput; //correction value
  analogWrite(engineL, constrain(speed + controlOutput, 0, 255));
  analogWrite(engineR, constrain(speed - controlOutput, 0, 255));
}
