#include "ModifiedNewPing4\ModifiedNewPing4.h"
#include "Motor.h"
#include <Servo.h>

//Define is faster than global variables
#define ULTRA_SONIC_PIN 47
#define DISTANCE_IR_PIN A0
#define SERVO_MOTOR_PIN A2

volatile bool WAIT_FOR_INTERRUPT;
NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 300);
Servo servoMotor();

void SetupState()
{
	//Set pins 24 and 22 as output (enable signals
	//Pin 22 is the outside IR and 24 is inside IR
	DDRA = 0x05;    
	PORTA &= 0x00; //Clear output initially
	WAIT_FOR_INTERRUPT = true;
	//Set Distance IR PIN
	pinMode(DISTANCE_IR_PIN, INPUT);
	servoMotor.attach(SERVO_MOTOR_PIN)
}

void FindRamp(MotorDrive* motors)
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  float baseSpeed = 0.30;
  //consider precomputing this
  float correctionFactor = 0.05; //This control is subject to change
  while(currentDistance != 0)
  {
    DriveStraight(motors, forward);
    currentDistance = sonar.modified_ping_cm(motors); //This drives motors while it waits
    //We've drifted left
    if(currentDistance < referenceDistance)
    {
      UpdateMotors(motors, baseSpeed+correctionFactor, baseSpeed);
    }
    //We've drifted right
    else if(currentDistance > referenceDistance)
    {
      UpdateMotors(motors, baseSpeed, baseSpeed+correctionFactor);
    }
    else
    {
      UpdateMotors(motors, baseSpeed, baseSpeed);
    }
  }
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
		else if(val3 =< val2)
		{
			return val2;
		}
	}
	else
	{
		if(val2 =< val3)
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
	UpdateMotors(motors, 25, 25); //arbritrary
	int currentSensorReading = 0;
	int previousSensorReading = 0; 

	// Rate of change is increasing
	while((currentSensorReading - previousSensorReading) >= 0)
	{
		TurnLeft(motors);
		previousSensorReading = currentSensorReading;
		currentSensorReading =  IRMedianOfThree();
	}
	while((currentSensorReading - previousSensorReading) < 0)
	{
		TurnLeft(motors);
		previousSensorReading = currentSensorReading;
		currentSensorReading =  IRMedianOfThree();
	}
}

void UltrasonicTurn(MotorDrive* motors)
{
	unsigned long referenceDistance = sonar.ping_cm();
	servoMotor.write(0); //not sure what the angle should be so we just do this, could be 90, or 180 or somethign
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5;
	unsigned long maxTolerance = referenceDistance + 5;

	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		TurnLeft(motors);
		measuredDistance = sonar.ping_cm();
	}
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
	float baseSpeed = 0.30; //Tune this
	float correctionSpeed = 0.37;

	//Test Counter to break the cycle
	int counter = 0;

	while(WAIT_FOR_INTERRUPT && counter < 30000)
	{
		DriveStraight(motors, forward);
		proximityData = PINA;
		//Both Motors See Ramp
		if((proximityData & 0x0A) == 0x0A)
		{
			//Drive the MotorB harder
			UpdateMotors(motors, baseSpeed, correctionSpeed);
		}
		//Both Motors See Space
		else if ((proximityData & 0x0A) == 0)
		{
			//Drive MotorA harder
			UpdateMotors(motors, correctionSpeed, baseSpeed);
		}
		else
		{
			UpdateMotors(motors, baseSpeed, baseSpeed);
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
	MotorDrive* motors = InitMotorDrive();
	UpdateMotors(motors, 0.99, 0.99);

	//  delay(2000);

	//  for(int i = 0; i<30000; i++)
	//  {
	while(1)
	{
		DriveStraight(motors, forward);
	}
	//  }

	//DriveOnFlat(motors);


	CleanupMotorDrive(motors);
	while(1);
}

