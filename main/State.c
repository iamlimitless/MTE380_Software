#include <pins_arduino.h>
#include <binary.h>
#include "State.h"

void SetupState()
{
	//Set pins 24 and 22 as output (enable signals
	//Pin 22 is the outside IR and 24 is inside IR
	DDRA = 0x05;    
	PORTA &= 0x00; //Clear output initially
	WAIT_FOR_INTERRUPT = true;
}

/*
Correction methods to try
	- Constant correction. 
	- Exponentially increasing correction
*/
void DriveUpRamp(MotorDrive* motors)
{
	PORTA |= 0x05; //Enable Proximity IR Sensors
	char proximityData;
	float baseSpeed = 30;
	float correctionSpeed = 37;

	while(WAIT_FOR_INTERRUPT)
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
	}
	WAIT_FOR_INTERRUPT = true;
}