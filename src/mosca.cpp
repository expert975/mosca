/*
  Rodrigo Martins
  My first class - learning C++ and OOP
  PID control
*/

#include <Arduino.h>
#include <L293D.h>
#include <Ultrasonic.h>
#include <Servo.h>

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
		error = _feedbackInput;
    //feedbackInput = _feedbackInput;
    //error = setpoint - feedbackInput; //error in the contolled system
    time = millis();
    deltaTime = float(time - lastTime)/1000; //delta time in seconds

    P = kP*error;
    I = I + kI*error*deltaTime;
    D =  kD*(lastFeedbackInput - feedbackInput)/deltaTime;

    // if (I > outMax) I = outMax;
    // if (I < outMin) I = outMin;
    // I = constrain(I, outMin, outMax); //prevent reset windup, where I grows more
                                      //than the system can respond
    controlOutput = P + I + D;
		Serial.println(controlOutput);

    // if (controlOutput > outMax) controlOutput = outMax;
    // if (controlOutput < outMin) controlOutput = outMin;
    // controlOutput = constrain(controlOutput, outMin, outMax); //restrict PID output to control output

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
void stopEngines();
void move(int, unsigned int);
void turn(int, int, unsigned int);
void walkToBall();
void summonTheClaw();
void lowerClaw();
void closeClaw();
void liftClaw();
void openClaw();
void checkForBall();
void considerRemainingTime();
void lockClaw();
void openBackdoor();
void closeBackdoor();
void walkToTriangle();
void checkForTriangle();
void halt();

//Macros
#define FOLLOWER 0
#define RESCUE 1

//Configs
#define OUTPUT_DEBUG 1
#define ENABLE_ENGINES 1
#define TARGET_SPEED 100 //how fast we should go
#define BALL_RADIUS 2
#define MAX_SCAN_COUNT 20 //how long it should look for balls before giving up

PID mosca(.5, 0, 0); //start PID object
L293D engR(44, 31, 33); //engineR object
L293D engL(45, 34, 32); //engineL object
Ultrasonic uLeft(39,38); // (Trig PIN,Echo PIN)
Ultrasonic uRight(52,53); // (Trig PIN,Echo PIN)
Ultrasonic uUp(43,42); // (Trig PIN,Echo PIN)
Ultrasonic uCenter(51,50); // (Trig PIN,Echo PIN)
Ultrasonic uBottom(49,48); // (Trig PIN,Echo PIN)
Servo frontServo;  //servoObjects
Servo clawServo; //servoObjects
Servo backServo; //servoObjects


//Sensor input
byte frontSensorPin = A10;
byte sensorOutRPin = A11;
byte sensorInRPin = A12;
byte sensorCenterPin = A13;
byte sensorInLPin = A14;
byte sensorOutLPin = A15;

byte button1 = A0;
byte button2 = A1;
byte button3 = A2;

int sensorOutR = 0;
int sensorInR = 0;
int sensorCenter = 0;
int sensorInL = 0;
int sensorOutL = 0;
int frontSensor = 0;

byte speedL = 0; //right engine speed
byte speedR = 0; //left engine speed

int distance; //stores distances within functions, tmp variable
int scanCounter = 0;
int lostDirection = 0;
byte modusOperandi; //defines how the system should behave (i.e. current mode)
// byte controlOutput; //correction value
float feedbackInput;

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

	pinMode(button1, INPUT);
	pinMode(button2, INPUT);
	pinMode(button3, INPUT);

	clawServo.attach(2);
	frontServo.attach(3);
	backServo.attach(4);

	mosca.outMin = -255;
	mosca.outMax = 255;

  mosca.setpoint = 0;

	Serial.begin(115200);

	modusOperandi = RESCUE; //defines how the system should behave (i.e. current mode)
	setClock(50);
	clockEnabled = true;
	definedClockTime = true;
	lastClockCycleTime = 0;
	clockCycleStartTime = 0;
	longExecutionTime = false;
  closeClaw();
}

void loop ()
{
	clockCycleStartTime = millis(); //clock cycle start


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
	// engL.set(255);
	// engR.set(255);

}

void calcEnginePID()
{
	feedbackInput = sensorInR - sensorInL;
	mosca.calculate(feedbackInput);
	speedL = constrain(TARGET_SPEED + mosca.controlOutput, 0, 255);
	speedR = constrain(TARGET_SPEED - mosca.controlOutput, 0, 255);
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
		sprintf(debugBuffer, "F: %04d, OR: %04d, IR: %04d, C: %04d, IL: %04d, OL: %04d, CLK: %03d, FB: %03d, OUT: %03d, L: %03d, R: %03d", frontSensor, sensorOutR,  sensorInR, sensorCenter, sensorInL, sensorOutL, lastClockCycleTime, (int)feedbackInput, (int)mosca.controlOutput, speedL, speedR);

		Serial.println(debugBuffer);
	#endif
}

void moveEngines()
{
	#if ENABLE_ENGINES
		engL.set(speedL);
		engR.set(speedR);
	#endif
}

void checkObstacle()
{
  distance = uCenter.Ranging(CM);
}

void avoidObstacle()
{
  if (uCenter.Ranging(CM));
  {

  }
}

//Second arena
void moveCenter()
{
	move(0, 1000);
	turn(0, 255, 500);
	move(0, 1000);
}

void scanTriangle()
{
  turn(1, 255, 300);
  delay(300);
  checkForTriangle();
}

void checkForTriangle()
{
  distance = abs(uCenter.Ranging(CM) - uUp.Ranging(CM));//distance between ball and wall sensors
  if (distance > 12)
  {
    walkToTriangle();
  }
}

void walkToTriangle()
{
  distance = uCenter.Ranging(CM); //define initial distanceToTriangle

  while (distance > 15) //while not near enough
  {
    distance = uBottom.Ranging(CM); //keep checking
    engL.set(TARGET_SPEED);
    engR.set(TARGET_SPEED);
  }
  stopEngines();
  delay(500);
  turn(0, 255, 500); //turn 180
  delay(500);
  move(1, 500); //move backwards
  delay(500);
  openBackdoor();
  halt();
}

void scanBall()
{
	while(scanCounter < MAX_SCAN_COUNT)
	{
    turn(1, 255, 300);
    delay(300);
    checkForBall();
	}
}

void checkForBall()
{
	distance = abs(uCenter.Ranging(CM) - uBottom.Ranging(CM));//distance between ball and wall sensors
	if (distance > BALL_RADIUS)
	{
		walkToBall();
    scanCounter = 0;
	}
  else
  {
    scanCounter++;
  }
}

void walkToBall()
{
	distance = uBottom.Ranging(CM); //define initial distanceToBall

	while (distance > 2) //while not near enough
	{
		distance = uBottom.Ranging(CM); //keep checking
	}
	stopEngines();
	summonTheClaw();
}

//Feed foreward
void summonTheClaw()
{
	lowerClaw();
	delay(1000);
	closeClaw();
	delay(1000);
	liftClaw();
	delay(1000);
	openClaw();
	delay(1000);
	considerRemainingTime(); // or not
}

void considerRemainingTime()
{
	//HOW?
}

void openClaw()
{
	for (int pos = 0; pos <= 60; pos += 1)
	{ // goes from 0 degrees to +/-60 degrees
		clawServo.write(pos);              // tell servo to go to position in variable 'pos'
  	delay(15);                       // waits 15ms for the servo to reach the position
	}
}

void closeClaw()
{
	clawServo.write(-60);        // goes from 60 degrees to 0 degrees
	delay(15);  // tell servo to go to position in variable 'pos'
}

void lockClaw()
{
	frontServo.writeMicroseconds(1500);
}

void liftClaw()
{
	frontServo.writeMicroseconds(1200);
	delay(11000);
}

void lowerClaw()
{
	frontServo.writeMicroseconds(1700);
	delay(900);
}
void openBackdoor()
{
	for (int pos = 0; pos <= 60; pos += 1)
	{ // goes from 0 degrees to +/-60 degrees
		backServo.write(pos);              // tell servo to go to position in variable 'pos'
		delay(15);                       // waits 15ms for the servo to reach the position
	}
}
void closeBackdoor()
{
	backServo.write(-60);        // goes from 60 degrees to 0 degrees
	delay(15);  // tell servo to go to position in variable 'pos'
}

void turn(int isRight, int speed, unsigned int time)
{
	int l = TARGET_SPEED;
	int r = TARGET_SPEED;
	unsigned int initTime;

	//Go right
	if (isRight)
	{
		r = -r;
	} //Go left
	else
	{
		l = -l;
	}

	initTime = millis();
	while (millis() - initTime < time)
	{
		Serial.print( millis() - initTime < time);
		engL.set(TARGET_SPEED);
		engR.set(-TARGET_SPEED);
	}
	stopEngines();
}

void move(int isForeward, unsigned int time)
{
	int l = TARGET_SPEED;
	int r = TARGET_SPEED;
	unsigned int initTime;

	//Go backwards
	if (isForeward)
	{
		r = -r;
		l = -l;
	}

	initTime = millis();

	while (millis() - initTime < time)
	{
		Serial.print( millis() - initTime < time);

		engL.set(TARGET_SPEED);
		engR.set(TARGET_SPEED);
	}
	stopEngines();
}

void stopEngines()
{
	engL.set(0);
	engR.set(0);
}

void halt()
{
  while(true){;}
}
