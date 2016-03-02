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
#define SERVO_MOTOR_PIN 10 //Pretty sure this needs to be on a pwm pin
#define I2C_INTERRUPT_PIN 2

volatile bool WAIT_FOR_INTERRUPT;
unsigned long DISTANCE_TO_BASE;
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
	servoMotor.attach(SERVO_MOTOR_PIN);
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
  int baseSpeed = 77; // 30% duty cycle
  int correctedSpeed = 102; // 40%
  
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
    currentDistance = sonar.ping_cm();
  }
  //probably going to need to drive for a bit longer
  MotorsOff();
}

inline void IRGuidedTurn()
{
	int currentSensorReading = 0;
	int previousSensorReading = 0; 
	TurnLeft(173, 82); //about 30%
	// Rate of change is increasing
	while((currentSensorReading - previousSensorReading) >= 0)
	{
		previousSensorReading = currentSensorReading;
		currentSensorReading = 12.361 * pow(0.005 * IRMedianOfThree(), -1.09); //magic numbers from excel trendline 
	}
	while((currentSensorReading - previousSensorReading) < 0)
	{
		previousSensorReading = currentSensorReading;
		currentSensorReading = 12.361 * pow(0.005 * IRMedianOfThree(), -1.09); //magic numbers from excel trendline 
	}
	MotorsOff();
}

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

	while(WAIT_FOR_INTERRUPT && counter < 100000) //200000
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
}

inline void DriveDownRamp()
{
}

inline void FindBase()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 77; // 30% duty cycle
  int correctedSpeed = 102; // 40%
  
  DriveForward(baseSpeed, baseSpeed);

  //Second condition check is to ensure we don't roll over the unsigned interger
  while((referenceDistance - currentDistance) < 8 || (referenceDistance - currentDistance) > 3000)
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
    currentDistance = sonar.ping_cm();
  }
  DISTANCE_TO_BASE = currentDistance;
  MotorsOff();
}

inline void UltrasonicTurn()
{
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long referenceDistance = DISTANCE_TO_BASE; //This value is set in Find Base (with an adjustment based on geometry)
	servoMotor.write(0);
	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;

	TurnLeft(173, 82); //about 30%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		measuredDistance = sonar.ping_cm();
	}
	DISTANCE_TO_BASE = measuredDistance;
	MotorsOff();
}


inline void DriveToTarget()
{
	unsigned long currentDistance = DISTANCE_TO_BASE;
	DriveForward(77, 77); //Might want this to be way faster
	//5 here represents "Target is right infront of us initiate stop sequence"
	while(currentDistance > 5)
	{
		previousDistance = currentDistance;
		currentDistance = sonar.ping_cm();
		if(currentDistance > previousDistance)
		{
			MotorsOff();
			CorrectRightDrift(previousDistance);
			DriveForward(77, 77); //Might want this to be way faster
		}
		else if(currentDistance == 0)
		{
			MotorsOff();
			CorrectLeftDrift(previousDistance);
			DriveForward(77, 77); //Might want this to be way faster
		}
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
	unsigned ling measuredDistance = 0;
	TurnRight(82, 173); //about 30%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		measuredDistance = sonar.ping_cm();
	}
	MotorsOff();
}

inline void CorrectRightDrift(unsigned long referenceDistance)
{
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;
	unsigned ling measuredDistance = 0;
	TurnLeft(173, 82); //about 30%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		measuredDistance = sonar.ping_cm();
	}
	MotorsOff();
}

inline void DriveUntilStop()
{
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
	delay(7500);
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;

	//Test Counter to break the cycle
	unsigned long long counter = 0;
	DriveForward(77, 77);
	while(counter < 100000) //200000
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
	SetupMotors();
	SetupState();
}

void loop() 
{	
  delay(2000);
  DriveOnFlat();
  MotorsOff();

  while(1)
  {
  	volatile int x = 0;
  	x++;
  }
}

