#include "State.h"
#include "Motor.h"


void setup() 
{
	SetupMotors();
}

void loop() 
{
	MotorDrive* motors = InitMotorDrive();
	
	CleanupMotorDrive(motors);
	while(1);
}
