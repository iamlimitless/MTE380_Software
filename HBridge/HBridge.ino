#include <IRremote.h>


const int RECV_PIN = 2;
volatile int READ_IR = 0;

IRrecv irrecv(RECV_PIN);
decode_results IR_VALUE;

//Port registers are big endian (Pins 7 to 0) ie 10000000 sets 7 on and all the rest off.
void setup() 
{
    /*
    The choice of PORTH takes away (I believe) some of the available clock pins, if we need more clock pins. 
    We can consider moving this to another port. Im not sure if we need the pins (looks like theyre for pwm output with the clock)
    to use the timer as a raw timer (to trigger sensor reading interrupts).
    */
    //Set the data direction register (DDRx) of B
    DDRH = B01111010; // This sets digital pins 9,8,7,6 to output For our motors. Tx pin 16 also output. 
    PORTH &= B10000111;
    attachInterrupt(digitalPinToInterrupt(RECV_PIN), handleIR, RISING);
    irrecv.enableIRIn(); // Start the receiver
    Serial.begin(9600);
}

//Could deffinitely try tweaking the delays by an offset to compensate for function overhead (thus reducing drift caused by motors)
void pwmToMotor(unsigned int frequency, float percentDutyA, float percentDutyB)
{
    // 1000ms/ (freq(in herz) / 1000)
    int time = 1000000 / frequency;

    if(percentDutyA > percentDutyB)
    {
        unsigned int firstDelay = percentDutyB * time;
        unsigned int secondDelay = (percentDutyA * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;

        PORTH |= B00101000; //Pulse on Pins 8, 6
        delayMicroseconds(firstDelay);
        PORTH &= B11011111; //Turn off pin 8
        delayMicroseconds(secondDelay);
        PORTH &= B11110111; //Turn off pin 6
        delayMicroseconds(timeRemaining);
    }
    else //PercentB >= PercentA
    {
        unsigned int firstDelay = percentDutyA * time;
        unsigned int secondDelay = (percentDutyB * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;

        PORTH |= B00101000; //Pulse on Pins 8, 6
        delayMicroseconds(firstDelay);
        PORTH &= B11110111; //Turn off pin 6
        delayMicroseconds(secondDelay); 
        PORTH &= B11011111; //Turn off pin 8
        delayMicroseconds(timeRemaining);
    }
}

void driveForward(unsigned int frequency, float percentDutyA, float percentDutyB)
{
    // Clear 9-6
    PORTH &= B10000111;
    pwmToMotor(frequency, percentDutyA, percentDutyB);
}

void handleIR()
{
    READ_IR = 1;
}

bool GetIRValue()
{
    int value = irrecv.decode(&IR_VALUE);
    switch(value)
    {
    case 1: //Set to a value
        return true;
    default:
        return false;
    }
}

void loop() 
{
    // bool driveMotor = false;
    // if(READ_IR) 
    // {
    //     READ_IR = 0;
    //     bool driveMotor = GetIRValue();
    // }
    
   /// delay(2000);  
   // for(int i = 0; i < 1000; i++)
  ///  {
    while(1)
    {
          driveForward(1000, 0.75, 0.60);
    }
 ///   }

    PORTH &= B10000111;

    while(true)
    {
      delay(1000);
    }
}





