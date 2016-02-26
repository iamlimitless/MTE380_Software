#include "State.h"
#include "Motor.h"


void setup() 
{
	SetupMotors();
	SetupState();
}

void loop() 
{
	MotorDrive* motors = InitMotorDrive();
	
	DriveUpRamp(motors);

	CleanupMotorDrive(motors);
	while(1);
}
