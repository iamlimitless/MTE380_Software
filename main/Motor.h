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

void SetupMotors();

void MotorsOff();

void DriveForward(int dutyA, int dutyB);

void DriveBackward(int dutyA, int dutyB);

void TurnLeft(int dutyA, int dutyB);

void TurnRight(int dutyA, int dutyB);

#ifdef __cplusplus
}
#endif

#endif //MOTOR_H