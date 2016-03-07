#include "ModifiedNewPing4\ModifiedNewPing4.h"
#include "Motor.h"
#include "math.h"
#include <Wire.h>
#include <Servo.h>

//Accelerometer Register
#define ZAXIS_REGISTER 0x02
#define TILT_REGISTER 0x03
#define INTERRUPT_REGISTER 0x06
#define MODE_REGISTER 0x07
#define AUTOWAKE_REGISTER 0x08
#define TAP_DETECTION_REGISTER 0x09
#define TAP_DEBOUNCE_REGISTER 0x0A

#define ULTRA_SONIC_PIN 3
#define DISTANCE_IR_PIN A0
#define SERVO_MOTOR_PIN 46 
#define I2C_INTERRUPT_PIN 2
#define SERVO_ENABLE 4

const int ULTRASONIC_DELAY = 500;
volatile bool WAIT_FOR_INTERRUPT;
unsigned long DISTANCE_TO_BASE;
unsigned long DISTANCE_TO_RAMPCENTER;

NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 265);
Servo servoMotor;

void SetupState()
{
	//Set pins 24 and 22 as output (enable signals
	//Pin 22 is the outside IR and 24 is inside IR
	DDRA = 0x01;    
	PORTA &= 0x00; //Clear output initially
	WAIT_FOR_INTERRUPT = true;
	//Set Distance IR PIN
	pinMode(DISTANCE_IR_PIN, INPUT);
	pinMode(SERVO_ENABLE, OUTPUT);
	Wire.begin();
	SetupAccelerometer();
}

inline void SetupAccelerometer()
{
	WriteAccelerometer(MODE_REGISTER, 0x40); //Set device into standby for register programming
	WriteAccelerometer(INTERRUPT_REGISTER, 0x04); //Enable interupts on tap
	WriteAccelerometer(AUTOWAKE_REGISTER, 0x60); // Filter 4 matching samples (at 120hz), before updating
	WriteAccelerometer(TAP_DETECTION_REGISTER, 0x60); // Detect only in the Z axis, threshold is +-1
	WriteAccelerometer(TAP_DEBOUNCE_REGISTER, 0xFF); // 0xFF, debouncing for days 0x00 is two tests in a row before interrupt is triggered (this is the minimum), 0x80 is 128
	//It is important to set operation mode last. See datasheet for rational
	WriteAccelerometer(MODE_REGISTER, 0x41); //Set active mode
	attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, FALLING);
}

inline void WriteAccelerometer(char reg, char data)
{
	Wire.beginTransmission(0x4C);
	Wire.write(reg); 
	Wire.write(data);
	int transmissionStatus = Wire.endTransmission(true);
  Serial.println(transmissionStatus);
}

//Note we should make sure passing a char is fine as opposed to byte
inline char ReadAccelerometer(char reg)
{
	Wire.beginTransmission(0x4C);
	Wire.write(reg);
	int restartStatus = Wire.endTransmission(false);
 	int numBytes = Wire.requestFrom(0x4C, 1, true);
	return Wire.read();
}

inline int IRMedianOfThree()
{
	//Start off greedy, if we get two values its right
	int val1 = analogRead(DISTANCE_IR_PIN);
	int val2 = analogRead(DISTANCE_IR_PIN);
	if(val1 == val2)
	{
		return val1;
	}

	int val3 = analogRead(DISTANCE_IR_PIN);

	if(val1 > val2)
	{
		if(val3 >= val1)
		{
			return val1;
		}
		else if(val3 <= val2)
		{
			return val2;
		}
	}
	else
	{
		if(val2 <= val3)
		{
			return val2;
		}
		else if(val1 >= val3)
		{
			return val1;
		}
	}
	return val3;
}


inline void FindRamp()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 115; // 77 
  int correctedSpeed = 130; // 97

  DriveForward(baseSpeed, baseSpeed);

  while(currentDistance != 0)
  {
    //We've drifted left
    if(currentDistance < referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
    //We've drifted right
    else if(currentDistance > referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    else
    {
	  DriveForward(baseSpeed, baseSpeed);
    }
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }
  //probably going to need to drive for a bit longer
  MotorsOff();
}

inline void UltrasonicTurn()
{
	digitalWrite(SERVO_ENABLE, HIGH);
  	servoMotor.attach(SERVO_MOTOR_PIN);
  	servoMotor.write(87);
  	delay(500); // Allows the servo to turn

    delayMicroseconds(ULTRASONIC_DELAY);  
    DISTANCE_TO_RAMPCENTER = sonar.ping_cm() + 9;
    unsigned long referenceDistance = 243 - DISTANCE_TO_RAMPCENTER; //240 - 9 - sonar.ping_cm();

  	servoMotor.write(181);
    int minTolerance = referenceDistance - 10; 
    int maxTolerance = referenceDistance + 10;
    unsigned long measuredDistance = 0;

	delay(500); // Allows the servo to turn //maybe more?

	TurnLeft(197, 57); 
    delay(450); // Lets the car start to turn before we read

    while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm(); //ping_cm()
	}
	// Probably want to set a global variable for the control distance (maybe not cause its a known part of the course)
	MotorsOff();
}

inline void DriveToRamp()
{
	unsigned long referenceDistance = DISTANCE_TO_RAMPCENTER;
	unsigned long currentDistance = referenceDistance;
	int baseSpeed = 115;
	int correctedSpeed = 140;
	DriveForward(baseSpeed, baseSpeed);

	while(1) //WAIT_FOR_INTERUPT
	{
	 	//We've drifted left
	    if(currentDistance < referenceDistance)
	    {
	      DriveForward(baseSpeed, correctedSpeed);
	    }
	    //We've drifted right
	    else if(currentDistance > referenceDistance)
	    {
	      DriveForward(correctedSpeed, baseSpeed);
	    }
	    else
	    {
		  DriveForward(baseSpeed, baseSpeed);
	    }
	    delayMicroseconds(ULTRASONIC_DELAY);  
	    currentDistance = sonar.ping_cm();
	}
	WAIT_FOR_INTERRUPT = true;
}

//Probably change this to ultrasonic drive straight
inline void IRDriveStraight()
{
  unsigned long referenceDistance = IRMedianOfThree();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 77; //33% duty cycle
  int correctedSpeed = 102; //This control is subject to change
  
  DriveForward(baseSpeed, baseSpeed);

  while(WAIT_FOR_INTERRUPT)
  {
    //We've drifted left
    if(currentDistance < referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
    //We've drifted right
    else if(currentDistance > referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    else
    {
	  DriveForward(baseSpeed, baseSpeed);
    }
    currentDistance = IRMedianOfThree();
  }
  WAIT_FOR_INTERRUPT = true;
}


/*
Correction methods to try
  - Constant correction. 
  - Exponentially increasing correction
*/
inline void DriveOnFlat()
{
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 166; //flat was 77
	int correctionSpeed = 217; //flat was 127

	//Test Counter to break the cycle
	unsigned long long counter = 0;

	DriveForward(baseSpeed, baseSpeed);

	while(WAIT_FOR_INTERRUPT && counter < 1200000) //200000
	{
		proximityData = PINA;
   
		switch((proximityData & 0x0A))
		{
			//Both Motors See Ramp
			case 0x00:
        //Drive the MotorB harder
        DriveForward(baseSpeed, correctionSpeed);
				break;
			//Both Motors See Open
			case 0x0A:
        //Drive MotorA harder
        DriveForward(correctionSpeed, baseSpeed);
				break;
			default:
				DriveForward(baseSpeed, baseSpeed);
		}

		counter++; //This is for test 
	}
	WAIT_FOR_INTERRUPT = true;
}

inline void DriveUpRamp()
{
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 166; //flat was 77
	int correctionSpeed = 217; //flat was 127

	//Test Counter to break the cycle
	unsigned long long counter = 0;

	DriveForward(baseSpeed, baseSpeed);

	while(WAIT_FOR_INTERRUPT && counter < 1300000) //200000
	{
		proximityData = PINA;
   
		switch((proximityData & 0x0A))
		{
			//Both Motors See Ramp
			case 0x00:
        //Drive the MotorB harder
        DriveForward(baseSpeed, correctionSpeed);
				break;
			//Both Motors See Open
			case 0x0A:
        //Drive MotorA harder
        DriveForward(correctionSpeed, baseSpeed);
				break;
			default:
				DriveForward(baseSpeed, baseSpeed);
		}

		counter++; //This is for test 
	}
	WAIT_FOR_INTERRUPT = true;
}

inline void DriveDownRamp()
{
}

inline void FindBase()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 128; 
  int correctedSpeed = 143;
  
  DriveForward(baseSpeed, baseSpeed);

  //Second condition check is to ensure we don't roll over the unsigned interger
  // while((referenceDistance - currentDistance) < 30 || (currentDistance > (referenceDistance + 5)))
  while((referenceDistance - currentDistance) < 30 || (referenceDistance - currentDistance) > 400000)
  {
    //We've drifted left
    if(currentDistance < referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
    //We've drifted right
    else if(currentDistance > referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    else
    {
	  DriveForward(baseSpeed, baseSpeed);
    }
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }
  
  DISTANCE_TO_BASE = currentDistance - 9; //Offset based on the distance.
  MotorsOff();
}

inline void TurnToTarget()
{
	digitalWrite(SERVO_ENABLE, HIGH);
  	servoMotor.attach(SERVO_MOTOR_PIN);
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long referenceDistance = DISTANCE_TO_BASE; //This value is set in Find Base (with an adjustment based on geometry)
	servoMotor.write(87);
	delay(500);
	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;

	TurnLeft(197, 57); 
    delay(330);
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
	DISTANCE_TO_BASE = measuredDistance;
	MotorsOff();
}


inline void DriveToTarget()
{
	servoMotor.write(3);
	delay(500);

	unsigned long referenceDistance = sonar.ping_cm();

	  int baseSpeed = 128; 
	  int correctedSpeed = 143; //should have been 143

	  //Might help?
	  while(referenceDistance == 0)
	  {
		TurnLeft(197, 57); 
	  	delayMicroseconds(ULTRASONIC_DELAY);
	  	referenceDistance = sonar.ping_cm();
	  }
	  
	  unsigned long currentDistance = referenceDistance;
	DriveForward(baseSpeed, baseSpeed); //Might want this to be way faster

  while(1)
  {

    if(currentDistance < referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    else if(currentDistance > referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
    //We've drifted right
    else
    {
	  DriveForward(baseSpeed, baseSpeed);
    }
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }

	DriveUntilStop();
}

/*
* Try out correcting with a drifting method (similar to the drive straight algorithms)
*/
inline void CorrectLeftDrift(unsigned long referenceDistance)
{
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;
	unsigned long measuredDistance = 0;
	TurnRight(82, 173); //about 30%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
	DISTANCE_TO_BASE = measuredDistance;
	MotorsOff();
}

inline void CorrectRightDrift(unsigned long referenceDistance)
{
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;
	unsigned long measuredDistance = 0;
	TurnLeft(173, 82); //about 30%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
	DISTANCE_TO_BASE = measuredDistance;
	MotorsOff();
}

inline void DriveUntilStop()
{
	//Consider using delay
	for(unsigned long i = 0; i<5000; i++)
	{
		//do nothing
	}
	MotorsOff();
}

void handleI2CInterrupt()
{
	WAIT_FOR_INTERRUPT = false;
}

void ConstructionCheck()
{
	//Test Motors
	delay(2000);
	DriveForward(77, 77);
	delay(2000);
	MotorsOff();
	delay(2000);

	//Test Ultrasonic
	unsigned long measuredDistance = 300;
	DriveForward(77, 77);
	while(measuredDistance > 60 || measuredDistance == 0)
	{
		measuredDistance = sonar.ping_cm();
	}
	MotorsOff();

	//IR Testing
	delay(10000);
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;

	//Test Counter to break the cycle
	unsigned long long counter = 0;
	DriveForward(77, 77);
	while(counter < 300000) //200000
	{
		proximityData = PINA;
		switch((proximityData & 0x0A))
		{
			//Both Motors See Ramp
		case 0x00:
        	//Drive the MotorB harder
        	DriveForward(0, 77);
			break;
		//Both Motors See Open
		case 0x0A:
	        //Drive MotorA harder
	        DriveForward(77, 0);
			break;
		default:
			DriveForward(77, 77);
		}
		counter++; //This is for test 
	}
	MotorsOff();
}

void setup() 
{
  Serial.begin(9600);
	SetupMotors();
	SetupState();
}

void loop() 
{	
 	delay(2000);
	digitalWrite(SERVO_ENABLE, HIGH);
 	servoMotor.attach(SERVO_MOTOR_PIN);
 	servoMotor.write(181);
   delay(1000);


  FindRamp();
  delay(2000);
  UltrasonicTurn();
   delay(2000);
   DriveToRamp();



  MotorsOff();
  
  while(1)
  {
  	volatile int x = 0;
  	x++;
  }
}

