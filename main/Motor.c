#include <pins_arduino.h>
#include <binary.h>
#include <stddef.h>
#include "Motor.h"


void SetupMotors()
{
    /*
    The choice of PORTH takes away (I believe) some of the available clock pins, if we need more clock pins. 
    We can consider moving this to another port. Im not sure if we need the pins (looks like theyre for pwm output with the clock)
    to use the timer as a raw timer (to trigger sensor reading interrupts).
    */
    //Set the data direction register (DDRx) of B
    DDRH = B01111010; // This sets digital pins 9,8,7,6 to output For our motors. Tx pin 16 also output. 
    PORTH &= B10000111;  
}

MotorDrive* InitMotorDrive()
{
  MotorDrive* newMotor = malloc(sizeof(MotorDrive));
  newMotor->motorADutyCycle = 0;
  newMotor->motorBDutyCycle = 0;
  return newMotor;
}

inline void UpdateMotors(MotorDrive* motor, float dutyA, float dutyB)
{
    motor->motorADutyCycle = dutyA;
    motor->motorBDutyCycle = dutyB;
}

void MotorsOff()
{
    // Clear Pins 9-6
    PORTH &= B10000111;
}

/*
Can maybe consolidate code for forward, reverse, and turning by
bitshifting bits based on the mode. Analyze the groups of bits,
and find a pattern
*/

inline void DriveStraight(MotorDrive* motor, MotorDirection direction)
{
    // Clear Pins 9-6
    PORTH &= B10000111;
    // 1000ms / (freq(in herz) / 1000) we choose a frequency of 10kHz
    int time = 100;
    char pulseBoth, disableA, disableB;

    if(direction == forward)
    {
        pulseBoth = B00101000;
        disableA = B11110111;
        disableB = B11011111;
    }
    else
    {
        pulseBoth = B01010000;
        disableA = B11101111;
        disableB = B10111111;
    }

    if(motor->motorADutyCycle > motor->motorBDutyCycle)
    {
        unsigned int firstDelay = motor->motorBDutyCycle * time;
        unsigned int secondDelay = (motor->motorADutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 8, 6 if forward, 9,7 if reverse
        delayMicroseconds(firstDelay);
        PORTH &= disableB; //Turn off pin 8, or 9 if reverse
        delayMicroseconds(secondDelay);
        PORTH &= disableA; //Turn off pin 6 or 7 if reverse
        delayMicroseconds(timeRemaining);
    }
    else //PercentB >= PercentA
    {
        unsigned int firstDelay = motor->motorADutyCycle * time;
        unsigned int secondDelay = (motor->motorBDutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 8, 6 if forward, 9,7 if reverse
        delayMicroseconds(firstDelay);
        PORTH &= disableA; //Turn off pin 6 or 7 if reverse
        delayMicroseconds(secondDelay); 
        PORTH &= disableB; //Turn off pin 8 or 9 if reverse
        delayMicroseconds(timeRemaining);
    }
}

void TurnLeft(MotorDrive* motor)
{
    PORTH &= B10000111;
    // 1000ms / (freq(in herz) / 1000) we choose a frequency of 10kHz
    int time = 100;
    char pulseBoth = B00110000;
    char disableA = B11101111;
    char disableB = B11011111;

    if(motor->motorADutyCycle > motor->motorBDutyCycle)
    {
        unsigned int firstDelay = motor->motorBDutyCycle * time;
        unsigned int secondDelay = (motor->motorADutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 9, 6
        delayMicroseconds(firstDelay);
        PORTH &= disableB; //Turn off pin 9
        delayMicroseconds(secondDelay);
        PORTH &= disableA; //Turn off pin 6
        delayMicroseconds(timeRemaining);
    }
    else //PercentB >= PercentA
    {
        unsigned int firstDelay = motor->motorADutyCycle * time;
        unsigned int secondDelay = (motor->motorBDutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 9, 6
        delayMicroseconds(firstDelay);
        PORTH &= disableA; //Turn off pin 6
        delayMicroseconds(secondDelay); 
        PORTH &= disableB; //Turn off pin 8
        delayMicroseconds(timeRemaining);
    }
}

void TurnRight(MotorDrive* motor)
{
 PORTH &= B10000111;
    // 1000ms / (freq(in herz) / 1000) we choose a frequency of 10kHz
    int time = 100;
    char pulseBoth = B01001000;
    char disableA = B11110111;
    char disableB = B10111111;

    if(motor->motorADutyCycle > motor->motorBDutyCycle)
    {
        unsigned int firstDelay = motor->motorBDutyCycle * time;
        unsigned int secondDelay = (motor->motorADutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 9, 6
        delayMicroseconds(firstDelay);
        PORTH &= disableB; //Turn off pin 9
        delayMicroseconds(secondDelay);
        PORTH &= disableA; //Turn off pin 6
        delayMicroseconds(timeRemaining);
    }
    else //PercentB >= PercentA
    {
        unsigned int firstDelay = motor->motorADutyCycle * time;
        unsigned int secondDelay = (motor->motorBDutyCycle * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;
        PORTH |= pulseBoth; //Pulse on Pins 9, 6
        delayMicroseconds(firstDelay);
        PORTH &= disableA; //Turn off pin 6
        delayMicroseconds(secondDelay); 
        PORTH &= disableB; //Turn off pin 8
        delayMicroseconds(timeRemaining);
    }
}

void CleanupMotorDrive(MotorDrive* motor)
{
    free(motor);
    motor = NULL;
}

