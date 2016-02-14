#include <IRremote.h>

const int AIN1_P5 = 5;
const int AIN2_P6 = 6;
const int BIN1_P10 = 2;
const int BIN2_P11 = 3;
const int RECV_PIN = 10;

volatile int READ_IR = 0;

IRrecv irrecv(RECV_PIN);
decode_results IR_VALUE;

void setup() 
{
  pinMode(AIN1_P5, OUTPUT);
  pinMode(AIN2_P6, OUTPUT);
  pinMode(BIN1_P10, OUTPUT);
  pinMode(BIN2_P11, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RECV_PIN), handleIR, RISING);
  irrecv.enableIRIn(); // Start the receiver
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

        //TODO Consider making this a function. Is the overhead worth the reuse?
        digitalWrite(AIN1_P5, HIGH);
        digitalWrite(BIN1_P10, HIGH);
        delayMicroseconds(firstDelay);
        digitalWrite(BIN1_P10, LOW);
        delayMicroseconds(secondDelay);    
        digitalWrite(AIN1_P5, LOW);
        delayMicroseconds(timeRemaining);
    }
    else //PercentB >= PercentA
    {
        unsigned int firstDelay = percentDutyA * time;
        unsigned int secondDelay = (percentDutyB * time) - firstDelay;
        unsigned int timeRemaining = (time - firstDelay) - secondDelay;

        //TODO Consider making this a function. Is the overhead worth the reuse?
        digitalWrite(AIN1_P5, HIGH);
        digitalWrite(BIN1_P10, HIGH);
        delayMicroseconds(firstDelay);
        digitalWrite(AIN1_P5, LOW);
        delayMicroseconds(secondDelay); 
        digitalWrite(BIN1_P10, LOW);
        delayMicroseconds(timeRemaining);
    }
}

void driveForward(unsigned int frequency, float percentDutyA, float percentDutyB)
{
  digitalWrite(AIN2_P6, LOW); 
  digitalWrite(BIN2_P11, LOW); 
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
    delay(5000);
    bool driveMotor = true;  //until we get the ir working                                                                                                                        
    
    if(driveMotor)
    {
        for(int i = 0; i < 200; i++)
        {
            driveForward(1000, 0.2, 0.2);
        }
    }

    digitalWrite(AIN1_P5, LOW); 
    digitalWrite(AIN2_P6, LOW); 
    digitalWrite(BIN1_P10, LOW); 
    digitalWrite(BIN2_P11, LOW); 

    while(true)
    {
      delay(1000);
    }
}
