#include "ModifiedNewPing4\ModifiedNewPing4.h"
#include "Motor.h"
#include "math.h"
#include <Servo.h>

//Define is faster than global variables
#define ULTRA_SONIC_PIN 47
#define DISTANCE_IR_PIN A0
#define SERVO_MOTOR_PIN 10 //PRetty sure this needs to be on a pwm pin

volatile bool WAIT_FOR_INTERRUPT;
NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 300);
Servo servoMotor;

void SetupState()
{
	//Set pins 24 and 22 as output (enable signals
	//Pin 22 is the outside IR and 24 is inside IR
	DDRA = 0x05;    
	PORTA &= 0x00; //Clear output initially
	WAIT_FOR_INTERRUPT = true;
	//Set Distance IR PIN
	pinMode(DISTANCE_IR_PIN, INPUT);
	servoMotor.attach(SERVO_MOTOR_PIN);
}

void FindRamp()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 85; //33% duty cycle
  int correctedSpeed = 95; //This control is subject to change
  
  DriveForward(baseSpeed, baseSpeed);

  while(currentDistance != 0)
  {
    currentDistance = sonar.ping_cm(); //This drives motors while it waits
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
  }
  MotorsOff();
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

void IRGuidedTurn(MotorDrive* motors)
{
	int currentSensorReading = 0;
	int previousSensorReading = 0; 
	TurnLeft(65, 65); //about 25%
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

void UltrasonicTurn(MotorDrive* motors)
{
	unsigned long referenceDistance = sonar.ping_cm();
	servoMotor.write(0); //not sure what the angle should be so we just do this, could be 90, or 180 or somethign
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5;
	unsigned long maxTolerance = referenceDistance + 5;

	TurnLeft(65, 65); //about 25%
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		measuredDistance = sonar.ping_cm();
	}
	MotorsOff();
}

/*
Correction methods to try
  - Constant correction. 
  - Exponentially increasing correction
*/
void DriveOnFlat(MotorDrive* motors)
{
	PORTA |= 0x05; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 85; // ~33%
	int correctionSpeed = 95; // ~37%

	//Test Counter to break the cycle
	int counter = 0;

	DriveForward(baseSpeed, baseSpeed);

	while(WAIT_FOR_INTERRUPT && counter < 30000)
	{
		proximityData = PINA;
		switch((proximityData & 0x0A))
		{
			//Both Motors See Ramp
			case 0x0A:
				//Drive the MotorB harder
				DriveForward(baseSpeed, correctionSpeed);
				break;
			//Both Motors See Space
			case 0x00:
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

void DriveUpRamp(MotorDrive* motors)
{
}

void DriveDownRamp(MotorDrive* motors)
{
}

void setup() 
{
	SetupMotors();
	SetupState();
}

void loop() 
{	

	 // delay(2000);

	DriveForward(123, 123); //about 50%

	//DriveOnFlat(motors);


	while(1);
}

