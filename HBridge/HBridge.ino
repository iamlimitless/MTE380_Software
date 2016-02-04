

const int AIN1_P5 = 5;
const int AIN2_P6 = 6;
const int BIN1_P10 = 10;
const int BIN2_P11 = 11;


void setup() 
{
  pinMode(AIN1_P5, OUTPUT);
  pinMode(AIN2_P6, OUTPUT);
  pinMode(BIN1_P10, OUTPUT);
  pinMode(BIN2_P11, OUTPUT);
}

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

void loop() 
{
  driveForward(1000, 0.5, 0.5);
}
