#ifndef MOTOR_H
#define MOTOR_H

/*
Some motor details
LeftMotor has A1 = Pin 6, A2 = Pin 7
Right Motor has B1 = Pin 8, B2 = Pin 9
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MotorDirection
{
	forward,
	reverse
} MotorDirection;

typedef struct MotorDrive 
{
	float motorADutyCycle;
	float motorBDutyCycle;
} MotorDrive;

void SetupMotors();

MotorDrive* InitMotorDrive();

void UpdateMotors(MotorDrive* motor, float dutyA, float dutyB);

void MotorsOff();

void DriveForward(MotorDrive* motor, MotorDirection direction);

void TurnLeft(MotorDrive* motor);

void TurnRight(MotorDrive* motor);

void CleanupMotorDrive(MotorDrive* motor);

#ifdef __cplusplus
}
#endif

#endif //MOTOR_H