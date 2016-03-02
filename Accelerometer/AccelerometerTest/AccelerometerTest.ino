
#define ZAXIS_REGISTER 0x02
#define TILT_REGISTER 0x03
#define INTERRUPT_REGISTER 0x06
#define MODE_REGISTER 0x07
#define AUTOWAKE_REGISTER 0x08
#define TAP_DETECTION_REGISTER 0x09
#define TAP_DEBOUNCE_REGISTER 0x0A

#include <Wire.h>

//Still need to understand auto sleep and autowake better
volatile boolean readData = false;
volatile int interruptCounter = 0;
const int I2C_INTERRUPT_PIN = 2;

void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	configureAccelerometer();
	attachInterrupt(digitalPinToInterrupt(I2C_INTERRUPT_PIN), handleI2CInterrupt, FALLING);
}

void configureAccelerometer()
{
	writeAccelerometer(MODE_REGISTER, 0x40); //Set device into standby for register programming
	writeAccelerometer(INTERRUPT_REGISTER, 0x04); //Enable interupts on tap
	writeAccelerometer(AUTOWAKE_REGISTER, 0x60); // Filter 4 matching samples (at 120hz), before updating
	writeAccelerometer(TAP_DETECTION_REGISTER, 0x60); // Detect only in the Z axis, threshold is +-1
	writeAccelerometer(TAP_DEBOUNCE_REGISTER, 0xFF); // 0xFF, debouncing for days 0x00 is two tests in a row before interrupt is triggered (this is the minimum), 0x80 is 128
	//It is important to set operation mode last. See datasheet for rational
	writeAccelerometer(MODE_REGISTER, 0x41); //Set active mode
}

void writeAccelerometer(byte reg, byte data)
{
	Wire.beginTransmission(0x4C); //Address is 0x4C shifted left 1 to add the r/w bit
	Wire.write(reg); //0x06
	Wire.write(data); //OxAA
	int transmissionStatus = Wire.endTransmission(true);
  Serial.println(transmissionStatus);
}

byte readAccelerometer(byte reg)
{
	Wire.beginTransmission(0x4C);
	Wire.write(reg);
	int restartStatus = Wire.endTransmission(false);
  int numBytes = Wire.requestFrom(0x4C, 1, true);
	return Wire.read();
}

void handleI2CInterrupt()
{
	readData = true;
	interruptCounter++;
}

void loop() 
{
	if(readData)
	{
		//Do some I2C stuff
    Serial.println("Counter value is");
    Serial.println(interruptCounter);
		readData = false;
    Serial.println(readAccelerometer(TILT_REGISTER));
	}
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
