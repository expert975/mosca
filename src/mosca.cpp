/*
  Rodrigo Martins
  My first class - learning C++ and OOP
  PID control
*/

#include <Arduino.h>
#include <L293D.h>

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

void moveEngines();
void readSensors();
void calcEnginePID();
void printSensors();
void modeManager();
void followerMode();
void rescueMode();
void clockManager();
void setClock(int);

//Macros
#define FOLLOWER 0
#define RESCUE 1

//Configs
#define OUTPUT_DEBUG 1
#define MOVE_ENGINES 1

PID mosca(0, 0, 0); //start PID object
L293D engR(44, 33, 31); //engineR object
L293D engL(45, 34, 32); //engineL object

//Sensor input
byte frontSensorPin = A10;
byte sensorOutRPin = A11;
byte sensorInRPin = A12;
byte sensorCenterPin = A13;
byte sensorInLPin = A14;
byte sensorOutLPin = A15;

int sensorOutR = 0;
int sensorInR = 0;
int sensorCenter = 0;
int sensorInL = 0;
int sensorOutL = 0;
int frontSensor = 0;

byte speedL = 0; //right engine speed
byte speedR = 0; //left engine speed

byte targetSpeed = 100; //how fast we should go
byte modusOperandi; //defines how the system should behave (i.e. current mode)
byte controlOutput; //correction value

//Clock variables
boolean clockEnabled; //enables the clock
unsigned int definedClockTime; //how long should each clock cycle take
unsigned int lastClockCycleTime; //stores the last clock cycle time
unsigned int clockCycleStartTime; //stores when last clock cycle started
boolean longExecutionTime; //holds true when loop takes longer than definedClockTime


char debugBuffer[200];

void setup()
{
  //Sensor input
  pinMode(sensorOutR, INPUT);
  pinMode(sensorInR, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorInL, INPUT);
  pinMode(sensorOutL, INPUT);

  mosca.setpoint = 0;

	Serial.begin(115200);

	modusOperandi = FOLLOWER; //defines how the system should behave (i.e. current mode)
	setClock(50);
	clockEnabled = true;
	definedClockTime = true;
	lastClockCycleTime = 0;
	clockCycleStartTime = 0;
	longExecutionTime = false;
}

void loop ()
{
	modeManager();
	clockManager();
}

//Allows modes to operate in fixed clock
void clockManager()
{
        //This next function should always be the last one in the loop sequence
        if (clockEnabled) //if current mode uses clock
        {
                lastClockCycleTime = millis() - clockCycleStartTime; //processing time used in the main loop
                if (lastClockCycleTime > definedClockTime) //if loop used processing time is greater than the defined clock time
                {
									longExecutionTime = true; //report long execution
                }
                else //if lastClockCycleTime was faster than definedClockTime...
                {
                        delay(definedClockTime - lastClockCycleTime); //...we still got some time to waste
                }
        }
}

//Sets clock time
void setClock(int clockTime)
{
        if (clockTime == 0)
        {
                clockEnabled = false;
                definedClockTime = 0;
        }
        else
        {
                clockEnabled = true;
                definedClockTime = clockTime;
        }
}

void modeManager ()
{
	switch (modusOperandi)
	{
		case FOLLOWER:
			followerMode();
			break;
		case RESCUE:
			rescueMode();
			break;
	}

	if (OUTPUT_DEBUG)
		printSensors();
}

void followerMode()
{
	readSensors();
	calcEnginePID();
	moveEngines();
}

void rescueMode()
{

}

void calcEnginePID()
{
	mosca.calculate(float(sensorInR - sensorInL));
	controlOutput = mosca.controlOutput;
	speedL = constrain(targetSpeed + controlOutput, 0, 255);
	speedR = constrain(targetSpeed - controlOutput, 0, 255);
}

void readSensors()
{
		sensorOutR = analogRead(sensorOutRPin);
		sensorInR = analogRead(sensorInRPin);
		sensorCenter = analogRead(sensorCenterPin);
		sensorInL = analogRead(sensorInLPin);
		sensorOutL = analogRead(sensorOutLPin);
		frontSensor = analogRead(frontSensorPin);
}

void printSensors()
{
	#if OUTPUT_DEBUG
		debugBuffer[0] = '\0';
		sprintf(debugBuffer, "F: %04d, OR: %04d, IR: %04d, C: %04d, IL: %04d, OL: %04d", frontSensor, sensorOutR,  sensorInR, sensorCenter, sensorInL, sensorOutL);
		Serial.println(debugBuffer);
	#endif
}

void moveEngines()
{
	#if MOVE_ENGINES
		engL.set(speedL);
		engR.set(speedR);
	#endif
}
