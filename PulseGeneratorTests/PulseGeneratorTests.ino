
#define MOTOR_A1_PIN 6
#define MOTOR_A2_PIN 7
#define MOTOR_B1_PIN 8
#define MOTOR_B2_PIN 9

void setup() 
{
	//Set timer to /8 prescale, operates in phaseCorrect mode therefor 16MHz / 8 / 510 = pwm frequency
	TCCR4B = (TCCR4B & 0xF8) | 0x02;
    DDRH = B01111010; // This sets digital pins 9,8,7,6 to output For our motors. Tx pin 16 also output. 
    PORTH &= B10000111;  
}

//Analog write takes a value between 0-255
inline void DriveMotor(int dutyA, int dutyB)
{
	PORTH &= B10000111;  
	analogWrite(MOTOR_A1_PIN, dutyA);
	analogWrite(MOTOR_B1_PIN, dutyB);
}

//We should verify that clock is at 16MHz
//We should test at max, min, and at various ranges to check the resolution
void loop()
{
	//33.333% duty cycle (Probably a bit less), expected frequency is 
	DriveMotor(0,0);
	while(1); //We should see the motors always driving
}
