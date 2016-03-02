#include <pins_arduino.h>
#include <binary.h>
#include <stddef.h>
#include "Motor.h"

/*
Some motor details
LeftMotor has A1 = Pin 6, A2 = Pin 7
Right Motor has B1 = Pin 8, B2 = Pin 9
*/
#define MOTOR_A1_PIN 6
#define MOTOR_A2_PIN 7
#define MOTOR_B1_PIN 8
#define MOTOR_B2_PIN 9

inline void SetupMotors()
{
    /*
    The choice of PORTH takes away (I believe) some of the available clock pins, if we need more clock pins. 
    We can consider moving this to another port. Im not sure if we need the pins (looks like theyre for pwm output with the clock)
    to use the timer as a raw timer (to trigger sensor reading interrupts).
    */
    //Set the data direction register (DDRx) of B
    DDRH = B01111010; // This sets digital pins 9,8,7,6 to output For our motors. Tx pin 16 also output. 
    PORTH &= B10000111;
    TCCR4B = ((TCCR4B & 0xF8) | 0x02);
}

inline void MotorsOff()
{
    PORTH &= B10101111;
    analogWrite(MOTOR_A1_PIN, 0);
    analogWrite(MOTOR_B1_PIN, 0);
}

inline void DriveForward(int dutyA, int dutyB)
{
    // Clear Pins 9-6 (To set enable signals)
    PORTH &= B10101111;
    analogWrite(MOTOR_A1_PIN, dutyA);
    analogWrite(MOTOR_B1_PIN, dutyB); 
}

//Note that this is in slow decay mode which is slighty different 
//Than forwards which is in fast decay mode
inline void DriveBackward(int dutyA, int dutyB)
{
    //Set appropriate enable signals
    PORTH = ((PORTH & B10101111) | B01010000);
    analogWrite(MOTOR_A1_PIN, dutyA);
    analogWrite(MOTOR_B1_PIN, dutyB); 
}

//Drive motor A backwards (Duty cycles on motors are reversed going back ie 0 is max)
inline void TurnLeft(int dutyA, int dutyB)
{
    //Set appropriate enable signals
    PORTH = ((PORTH & B10101111) | B00010000);
    analogWrite(MOTOR_A1_PIN, dutyA);
    analogWrite(MOTOR_B1_PIN, dutyB);
}

//Drive motor B  backwards (Duty cycles on motors are reversed going back ie 0 is max)
inline void TurnRight(int dutyA, int dutyB)
{
    //Set appropriate enable signals
    PORTH = ((PORTH & B10101111) | B01000000);
    analogWrite(MOTOR_A1_PIN, dutyA);
    analogWrite(MOTOR_B1_PIN, dutyB);
}
