
#define ZAXIS_REGISTER 0x02
#define INTERRUPT_REGISTER 0x06
#define MODE_REGISTER 0x07
#define AUTOWAKE_REGISTER 0x08
#define TAP_DETECTION_REGISTER 0x09
#define TAP_DEBOUNCE_REGISTER 0x0A

#include <Wire.h>

//Still need to understand auto sleep and autowake better

volatile boolean readData = false;
volatile int interruptCounter = 0;
const int I2C_INTERRUPT_PIN = 18;

void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	//configureAccelerometer();
	attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, RISING);
}

void configureAccelerometer()
{
  Serial.print("First Line in Configure");
	writeAccelerometer(MODE_REGISTER, 0x40); //Set device into standby for register programming
   Serial.print("Line after Write");

	writeAccelerometer(INTERRUPT_REGISTER, 0x04); //Enable interupts on tap
	writeAccelerometer(AUTOWAKE_REGISTER, 0x60); // Filter 4 matching samples (at 120hz), before updating
	writeAccelerometer(TAP_DETECTION_REGISTER, 0x60); // Detect only in the Z axis, threshold is +-1
	writeAccelerometer(TAP_DEBOUNCE_REGISTER, 0x00); // two tests in a row before interrupt is triggered (this is the minimum)
	//It is important to set operation mode last. See datasheet for rational
	writeAccelerometer(MODE_REGISTER, 0xC1); //Set active mode
}

void writeAccelerometer(byte reg, byte data)
{
	Wire.beginTransmission(0x98); //0x4C but should maybe be 0x98 because arduino sends the right bit.
	Wire.write(reg);
	Wire.write(data);
	int transmissionStatus = Wire.endTransmission(true);
	//Error Checking Code
	Serial.print("The trasmission status code is ");
	Serial.println(transmissionStatus);
}

byte readAccelerometer(byte reg)
{
	Wire.beginTransmission(0x4C); //0x4C but should maybe be 0x98 because arduino sends the right bit.
	Wire.write(reg);
	int restartStatus = Wire.endTransmission(false);
	Wire.beginTransmission(0x4C);
	byte data = Wire.read();
	int transmissionStatus = Wire.endTransmission(true);
}

void handleI2CInterrupt()
{
	readData = true;
	interruptCounter++;
}

void loop() 
{
  /*
	if(readData)
	{
		//Do some I2C stuff
    Serial.println("Interupt Called");
		Serial.println(readAccelerometer(ZAXIS_REGISTER));
		readData = false;
	}*/
 writeAccelerometer(INTERRUPT_REGISTER, 0xAA);
 delay(1000);
}

/*
I2C Communication

Each transmission consists of a START condition (Figure 7) sent by a master, followed by MMA7660FC's 7-bit slave address
plus R/W bit, a register address byte, one or more data bytes, and finally a STOP condition

MMA7660FC has a 7-bit long slave address, shown in Figure 11. The bit following the 7-bit slave address (bit eight) is the
R/W bit, which is low for a write command and high for a read command. The device has a factory set I2C slave address which
is normally 1001100 (0x4C)

In order to enable Tap detection in the device the user must enable the Tap Interrupt in the INTSU (0x06) register and AMSR
[2:0] = 000 in the SR (0x08) register. In this mode, TILT (0x03) register, XOUT (0x00), YOUT (0x01), and ZOUT (0x02) registers
will update at the 120 samples/second.
*/
