#include "ModifiedNewPing4\ModifiedNewPing4.h"
#include "Motor.h"
#include "math.h"
#include <Wire.h>
#include <Servo.h>

//Accelerometer Register
#define XAXIS_REGISTER 0x00
#define ZAXIS_REGISTER 0x02
#define TILT_REGISTER 0x03
#define SLEEP_COUNTER_REGISTER 0x05
#define INTERRUPT_REGISTER 0x06
#define MODE_REGISTER 0x07
#define AUTOWAKE_REGISTER 0x08
#define TAP_DETECTION_REGISTER 0x09
#define TAP_DEBOUNCE_REGISTER 0x0A

// Pin definitions
#define ULTRA_SONIC_PIN 3
#define DISTANCE_IR_PIN A0
#define US_SERVO_MOTOR_PIN 46 
#define US_SERVO_ENABLE 4
#define ACCEL_ENABLE 12
#define BRAKE_SERVO_MOTOR_PIN 44 
#define BRAKE_SERVO_ENABLE 5
#define I2C_INTERRUPT_PIN 2

boolean GLOBAL_DEBUG_FLAG = false;

// brake off is 36, break on is 44

const int ULTRASONIC_DELAY = 500;
unsigned long DISTANCE_TO_BASE;
unsigned long DISTANCE_TO_RAMPCENTER;

NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 265);
Servo usServoMotor;
Servo brakeServoMotor;

void SetupState()
{
	//Set pins 24 and 22 as output (enable signals
	//Pin 22 is the outside IR and 24 is inside IR
	DDRA = 0x01;    
	PORTA &= 0x00; //Clear output initially
	//Set Distance IR PIN
	pinMode(DISTANCE_IR_PIN, INPUT);
	pinMode(US_SERVO_ENABLE, OUTPUT);
	pinMode(BRAKE_SERVO_ENABLE, OUTPUT);
	pinMode(ACCEL_ENABLE, OUTPUT);
	Wire.begin();
}

//NEed to explicitly clear registers since there state is saved between reprograms
inline void SetupAccelerometer()
{
	WriteAccelerometer(MODE_REGISTER, 0x78); //Set device into standby for register programming
	WriteAccelerometer(SLEEP_COUNTER_REGISTER, 0xFF);
	WriteAccelerometer(INTERRUPT_REGISTER, 0x00);
	//It is important to set operation mode last. See datasheet for rational
	WriteAccelerometer(MODE_REGISTER, 0x79); //Set active mode, used to be 0x41
	// attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, FALLING);
}

inline void WriteAccelerometer(char reg, char data)
{
	Wire.beginTransmission(0x4C);
	Wire.write(reg); 
	Wire.write(data);
	int transmissionStatus = Wire.endTransmission(true);
	if(transmissionStatus != 0)
	{
		// Serial.println(transmissionStatus);
		// Serial.println(byte(reg));
		digitalWrite(53, HIGH);
	} 
}

//Note we should make sure passing a char is fine as opposed to byte
inline byte ReadAccelerometer(byte reg)
{
	Wire.beginTransmission(0x4C);
	Wire.write(reg);
	int restartStatus = Wire.endTransmission(false);
	int numBytes = Wire.requestFrom(0x4C, 1, true);
	return Wire.read();
}

inline void FindRamp()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 115;
  int correctedSpeed = 130;

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
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }
  //probably going to need to drive for a bit longer
  MotorsOff();
}

inline void UltrasonicTurn()
{
	digitalWrite(US_SERVO_ENABLE, HIGH);
  	usServoMotor.attach(US_SERVO_MOTOR_PIN);
  	usServoMotor.write(87);
  	delay(500); // Allows the servo to turn

    delayMicroseconds(ULTRASONIC_DELAY);  
    unsigned long rightBoundaryDistance = sonar.ping_cm();

    while(rightBoundaryDistance > 38)
    {
    	DriveForward(115, 115); // this is the base speed set in findRamp
    	delayMicroseconds(ULTRASONIC_DELAY);
		rightBoundaryDistance = sonar.ping_cm();
    }
    MotorsOff();

    delayMicroseconds(ULTRASONIC_DELAY);
    rightBoundaryDistance = sonar.ping_cm();
    unsigned long referenceDistance = 243 - rightBoundaryDistance - 9; //240 - 9 - sonar.ping_cm();
  	usServoMotor.write(181);
    int minTolerance = referenceDistance - 6; 
    int maxTolerance = referenceDistance + 6;
    unsigned long measuredDistance = 0;

	delay(500); // Allows the servo to turn

	TurnLeft(197, 63); 
    delay(450); // Lets the car start to turn before we read

    while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm(); //ping_cm()
	}
	// Probably want to set a global variable for the control distance (maybe not cause its a known part of the course)
	MotorsOff();
}

inline void DriveToRamp()
{
	unsigned long referenceDistance = 210;
	unsigned long currentDistance = referenceDistance;
	int baseSpeed = 76; //might want to make this a bit higher (83) in order to trigger the tap interrupt before stalling the motors
	int correctedSpeed = 127;
	DriveForward(baseSpeed, baseSpeed);

	// byte accelerometerData = ReadAccelerometer(ZAXIS_REGISTER);

	//while(accelerometerData < 48 || (accelerometerData & 0x40) == 0x40)
	while(1)
	{
	 	//We've drifted left
	    if(currentDistance > referenceDistance || currentDistance == 0)
	    {
	      DriveForward(baseSpeed, correctedSpeed);
	    }
	    //We've drifted right
	    else if(currentDistance < referenceDistance)
	    {
	      DriveForward(correctedSpeed, baseSpeed);
	    }
	    else
	    {
		  DriveForward(baseSpeed, baseSpeed);
	    }
	    delayMicroseconds(ULTRASONIC_DELAY);
	    // accelerometerData = ReadAccelerometer(ZAXIS_REGISTER);
	    currentDistance = sonar.ping_cm();
	}
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

	while(WAIT_FOR_INTERRUPT && counter < 1200000) //200000
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
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 166; //flat was 77
	int correctionSpeed = 217; //flat was 127

	//Test Counter to break the cycle
	unsigned long long counter = 0;

	DriveForward(baseSpeed, baseSpeed);

	// while(WAIT_FOR_INTERRUPT && counter < 1300000) //200000
	while(counter < 1300000)
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
	// WAIT_FOR_INTERRUPT = true;
}

inline void DriveDownRamp()
{
	digitalWrite(BRAKE_SERVO_ENABLE, HIGH);
 	brakeServoMotor.attach(BRAKE_SERVO_MOTOR_PIN);
 	brakeServoMotor.write(44); // 44
 	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 51; 
	int correctionSpeed = 89;
	
	//Test Counter to break the cycle
	unsigned long long counter = 0;
	
	DriveForward(baseSpeed, baseSpeed);

	// while(WAIT_FOR_INTERRUPT) //200000
	while(counter < 1300000)
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
}

inline void FindBase()
{
  unsigned long referenceDistance = sonar.ping_cm();
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 128; 
  int correctedSpeed = 143;
  
  DriveForward(baseSpeed, baseSpeed);

  //Second condition check is to ensure we don't roll over the unsigned interger
  // while((referenceDistance - currentDistance) < 30 || (currentDistance > (referenceDistance + 5)))
  while((referenceDistance - currentDistance) < 30 || (referenceDistance - currentDistance) > 400000)
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
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }
  
  DISTANCE_TO_BASE = currentDistance - 9; //Offset based on the distance.
  MotorsOff();
}

inline void TurnToTarget()
{
	digitalWrite(US_SERVO_ENABLE, HIGH);
  	usServoMotor.attach(US_SERVO_MOTOR_PIN);
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long referenceDistance = DISTANCE_TO_BASE; //This value is set in Find Base (with an adjustment based on geometry)
	usServoMotor.write(87);
	delay(500);
	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;

	TurnLeft(197, 57); 
    delay(330);
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
	DISTANCE_TO_BASE = measuredDistance;
	MotorsOff();
}


inline void DriveToTarget()
{
	usServoMotor.write(3);
	delay(500);

	unsigned long referenceDistance = sonar.ping_cm();

	  int baseSpeed = 128; 
	  int correctedSpeed = 143; //should have been 143

	  //Might help?
	  while(referenceDistance == 0)
	  {
		TurnLeft(197, 57); 
	  	delayMicroseconds(ULTRASONIC_DELAY);
	  	referenceDistance = sonar.ping_cm();
	  }
	  
	  unsigned long currentDistance = referenceDistance;
	DriveForward(baseSpeed, baseSpeed); //Might want this to be way faster

  while(1)
  {

    if(currentDistance < referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    else if(currentDistance > referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
    //We've drifted right
    else
    {
	  DriveForward(baseSpeed, baseSpeed);
    }
    delayMicroseconds(ULTRASONIC_DELAY);  
    currentDistance = sonar.ping_cm();
  }

	DriveUntilStop();
}


inline void DriveUntilStop()
{
	//Consider using delay
	for(unsigned long i = 0; i<5000; i++)
	{
		//do nothing
	}
	MotorsOff();
}

void setup() 
{
  	Serial.begin(9600);
  	//For testing only
  	pinMode(53, OUTPUT);
	SetupMotors();
	SetupState();
  	delay(2000);
}

void TestDrive()
{
	digitalWrite(ACCEL_ENABLE, HIGH);
	delay(1);
	SetupAccelerometer();

	DriveForward(83, 83);
	byte data = ReadAccelerometer(XAXIS_REGISTER);
	delay(110);
	while((data & 0x40) == 0x40)
	{
		data = ReadAccelerometer(XAXIS_REGISTER);
		delay(110);
	}

	int counter = 0;

	while(data > 58 || data < 40) //was 54
	{
		counter++;
		data = ReadAccelerometer(XAXIS_REGISTER);
		delay(110);
		while((data & 0x40) == 0x40)
		{
			data = ReadAccelerometer(XAXIS_REGISTER);
			delay(110);
		}

		if(counter > 10000)
		{
			digitalWrite(ACCEL_ENABLE, LOW);
			delayMicroseconds(100);
			digitalWrite(ACCEL_ENABLE, HIGH);
			delay(1);
			SetupAccelerometer();
			counter = 0;
		}
	}
	MotorsOff();
}

void loop() 
{	
   delay(2000);
//DriveDownRamp();
//	digitalWrite(US_SERVO_ENABLE, HIGH);
// 	usServoMotor.attach(US_SERVO_MOTOR_PIN);
// 	usServoMotor.write(181);
//  	delay(500);

//    digitalWrite(BRAKE_SERVO_ENABLE, HIGH);
//    brakeServoMotor.attach(BRAKE_SERVO_MOTOR_PIN);
//    brakeServoMotor.write(37);


  MotorsOff();
  
  while(1)
  {
  }
}

