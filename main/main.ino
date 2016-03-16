#include "ModifiedNewPing4\ModifiedNewPing4.h"
#include "Motor.h"
#include "math.h"
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

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
#define BRAKE_SERVO_MOTOR_PIN 44 
#define BRAKE_SERVO_ENABLE 5
#define I2C_INTERRUPT_PIN 2

// brake off is 36, break on is 44

const int ULTRASONIC_DELAY = 500; //us
const int ACCEL_DELAY = 5; //ms
unsigned long DISTANCE_TO_WOODWALL;
unsigned long DISTANCE_TO_BASE;
unsigned long DISTANCE_TO_RAMPCENTER;
unsigned long WALL_TO_BASE_CORRECTION;

NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 265);
Servo usServoMotor;
Servo brakeServoMotor;
MPU6050 IMU;

int16_t twosToDec(int16_t twosCompValue)
{
	int16_t decimalVal = (twosCompValue & 0x7FFF); //Everything but the top bit
	decimalVal -= (twosCompValue & 0x8000);
	return decimalVal;
}

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
	Wire.begin();
	IMU.initialize();
}

inline void DrivePastMagnetWall()
{
	//This code might not be needed
	digitalWrite(US_SERVO_ENABLE, HIGH);
	usServoMotor.attach(US_SERVO_MOTOR_PIN);
	usServoMotor.write(181);
	delay(500);

  unsigned long referenceDistance = sonar.ping_cm();
  DISTANCE_TO_WOODWALL = referenceDistance + 25;
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 115;
  int correctedSpeed = 163;
  DriveForward(baseSpeed, baseSpeed);

  while(referenceDistance == 0)
  {
  	delayMicroseconds(ULTRASONIC_DELAY);  
  	referenceDistance = sonar.ping_cm();
  }

  while(currentDistance < (referenceDistance + 15))
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
}

inline void FindRamp()
{
  unsigned long referenceDistance = DISTANCE_TO_WOODWALL;
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 115;
  int correctedSpeed = 163;

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

inline void CheckEarlyZero()
{
  usServoMotor.write(87);
  delay(500); // Allows the servo to turn

  delayMicroseconds(ULTRASONIC_DELAY);  
  unsigned long rightBoundaryDistance = sonar.ping_cm();

  while(rightBoundaryDistance > 38 || rightBoundaryDistance == 0)
  {
  	DriveForward(115, 115); // this is the base speed set in findRamp
   	delayMicroseconds(ULTRASONIC_DELAY);
	rightBoundaryDistance = sonar.ping_cm();
  }
  MotorsOff();
}

inline void DriveToRamp()
{
	usServoMotor.write(3);
	delay(500); // Allows the servo to turn

	unsigned long referenceDistance = 31; //BASED ON COMP
	unsigned long currentDistance = referenceDistance;
	int baseSpeed = 57; //might want to make this a bit higher (83) in order to trigger the tap interrupt before stalling the motors
	int correctedSpeed = 153; //Maybe we are seeing the wheel slip because it is too high try 115 (45%)
	DriveForward(baseSpeed, baseSpeed);

	double accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	while(accelerometerData > -0.35)
	{
	 	//We've drifted right
	  if(currentDistance < referenceDistance || currentDistance == 0)
	  {
	    DriveForward(baseSpeed, correctedSpeed);
	  }
	  //We've drifted left
	  else if(currentDistance > referenceDistance)
	  {
	    DriveForward(correctedSpeed, baseSpeed);
	  }
	  else
	  {
		  DriveForward(baseSpeed, baseSpeed);
	  }
		accelerometerData =  twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	  	//delay(ACCEL_DELAY); // see if we actually need this to trigger
	  	//What should the delay be?
		delayMicroseconds(ULTRASONIC_DELAY);
		currentDistance = sonar.ping_cm();
	}
}

inline void DriveUpRamp()
{
	//Disable Ultrasonic Servo.
	digitalWrite(US_SERVO_ENABLE, LOW);

	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 166; //166  204 with higher weight
	int correctionSpeed = 217; //217 255 with higher weight

	DriveForward(baseSpeed, baseSpeed);

	//Consider an accelerometer debounce period
	double accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	while(accelerometerData < -0.15)
	{
		proximityData = PINA;
   		switch((proximityData & 0x0A))
		{
		  //Both Sensors See Ramp
		  case 0x00:
	        //Drive MotorB harder
	        DriveForward(baseSpeed, correctionSpeed);
			break;
		  //Both Sensors See Open
		  case 0x0A:
	        //Drive MotorA harder
	        DriveForward(correctionSpeed, baseSpeed);
			break;
		  default:
			  DriveForward(baseSpeed, baseSpeed);
		}
		accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	}
	MotorsOff();
	digitalWrite(BRAKE_SERVO_ENABLE, HIGH);
	brakeServoMotor.attach(BRAKE_SERVO_MOTOR_PIN);
	brakeServoMotor.write(44);
	delay(300);
}

inline void DriveOnFlat()
{
	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 128;
	int correctionSpeed = 166;
	
	DriveForward(baseSpeed, baseSpeed);

	double accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	while(accelerometerData < 0.3)
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
		accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	}
	MotorsOff();
}

inline void DriveDownRamp()
{
 	PORTA |= 0x01; //Enable Proximity IR Sensors
	char proximityData;
	int baseSpeed = 51; 
	int correctionSpeed = 89;
	
	DriveForward(baseSpeed, baseSpeed);

	//Consider an accelerometer debounce period
	double accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	while(accelerometerData > 0.20)
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
		double accelerometerData = twosToDec(IMU.MPU6050::getRotationY()) / 16384.0;
	}
  MotorsOff();
  brakeServoMotor.write(37);
  delay(200);
  digitalWrite(BRAKE_SERVO_ENABLE, LOW);
}

inline void StraightenAfterRamp()
{
	//Reenable Ultrasonic Servo.
	digitalWrite(US_SERVO_ENABLE, HIGH);

	unsigned long referenceDistance = 31; //Based on actual comp distance
	unsigned long currentDistance = sonar.ping_cm();
	int baseSpeed = 83; //might want to make this a bit higher (83) in order to trigger the tap interrupt before stalling the motors
	int correctedSpeed = 134;

	DriveForward(baseSpeed, baseSpeed);

	int counter = 0;
	while(counter < 200)
	{
		counter++;
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
	  currentDistance = sonar.ping_cm();
	}
	MotorsOff();
	usServoMotor.write(87);
	delay(500);
}

inline void DriveToBackWall()
{
	unsigned long referenceDistance = sonar.ping_cm();
	while(referenceDistance > 20)
	{
		DriveForward(83, 83);
		delayMicroseconds(ULTRASONIC_DELAY);
		referenceDistance = sonar.ping_cm();
	}
	MotorsOff();
}

inline void DrivePastRamp()
{
	usServoMotor.write(181);
	delay(500);
	DriveForward(115, 115);
	unsigned long measuredDistance = 0;
	while(measuredDistance == 0)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
}

inline void FindBase()
{
  unsigned long referenceDistance = 220; //used to be sonar.ping_cm()
  unsigned long currentDistance = referenceDistance;
  int baseSpeed = 115; 
  int correctedSpeed = 204;
  
  DriveForward(baseSpeed, baseSpeed);

  //Second condition check is to ensure we don't roll over the unsigned interger
  // while((referenceDistance - currentDistance) < 30 || (currentDistance > (referenceDistance + 5)))
  while((referenceDistance - currentDistance) < 15 || (referenceDistance - currentDistance) > 400000 || currentDistance == 0)
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
  MotorsOff();
  delay(50);
  DISTANCE_TO_BASE = sonar.ping_cm() - 9; //Offset based on the distance.
}

inline void TurnToTarget()
{
	usServoMotor.write(87);
	// Might want a different ultrasonic to limit sight outside the bounday.Configure the ultrasonic to not see things within a certain distance.
	unsigned long referenceDistance = DISTANCE_TO_BASE; //This value is set in Find Base (with an adjustment based on geometry)
	delay(500);

	delayMicroseconds(ULTRASONIC_DELAY);
	WALL_TO_BASE_CORRECTION = sonar.ping_cm() + 9;

	unsigned long measuredDistance = 0;
	unsigned long minTolerance = referenceDistance - 5; //We should adjust the tolerances based on our ability to turn
	unsigned long maxTolerance = referenceDistance + 5;

	TurnLeft(166, 115);  // 179, 102
  delay(300);
	while(measuredDistance < minTolerance || measuredDistance > maxTolerance)
	{
		delayMicroseconds(ULTRASONIC_DELAY);
		measuredDistance = sonar.ping_cm();
	}
	MotorsOff();
}

inline void DriveToTarget()
{
	usServoMotor.write(3);
	delay(500);

	unsigned long referenceDistance = WALL_TO_BASE_CORRECTION;

	int baseSpeed = 115; 
	int correctedSpeed = 204;

	while(referenceDistance == 0)
	{
	  TurnLeft(197, 57); 
	  delayMicroseconds(ULTRASONIC_DELAY);
	  referenceDistance = sonar.ping_cm();
	}
	  
	unsigned long currentDistance = referenceDistance;
	DriveForward(baseSpeed, baseSpeed);

  while(1) 
  {
    //We've drifted right
    if(currentDistance < referenceDistance)
    {
      DriveForward(baseSpeed, correctedSpeed);
    }
    //We've drifted left
    else if(currentDistance > referenceDistance)
    {
      DriveForward(correctedSpeed, baseSpeed);
    }
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
	delay(500); //Tune this value
	MotorsOff();
}


void setup() 
{
  	Serial.begin(9600);
  	//For testing only
  	pinMode(53, OUTPUT);

	SetupMotors();
	SetupState();
}

inline void Turn90Left()
{
	double totalDegrees = 0;
	TurnLeft(179, 102);
	while(totalDegrees < 80)
	{
		totalDegrees += (twosToDec(IMU.getRotationZ()) / 131.0) * 0.01; // could optimize this to 0.01/131.0
    	delay(10);
	}
  MotorsOff();
}

//Make sure to rip out debug code (serial prints, and led stuff before the comp)
//Make sure to tune servo delays
//Make sure to tune all the drive speeds
void loop() 
{	
	delay(2000);
	//IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500); if were not seeing the turn work at the right speed
	Turn90Left();

	// digitalWrite(BRAKE_SERVO_ENABLE, HIGH);
	// brakeServoMotor.attach(BRAKE_SERVO_MOTOR_PIN);
	// brakeServoMotor.write(37);

  // DrivePastMagnetWall();
  // FindRamp();
  // CheckEarlyZero();
	// Turn90Left();
  // DriveToRamp();

  // DriveUpRamp();
  // digitalWrite(53, HIGH);
  // DriveOnFlat();
  // digitalWrite(53, LOW);
  // DriveDownRamp();
  // digitalWrite(53, LOW);



	// digitalWrite(US_SERVO_ENABLE, HIGH);
	// usServoMotor.attach(US_SERVO_MOTOR_PIN);
	// usServoMotor.write(3);

 //  StraightenAfterRamp(); //could consider increaing the distance/durations of this
	// DriveToBackWall();
	// Turn90Left();
 //  DrivePastRamp();
 //  FindBase();
 	// Turn90Left(); 	//  TurnToTarget(); replaced with 90 degree turn
 //  DriveToTarget();

	MotorsOff();
	while(1)
	{
		delay(5000);
	}
}

