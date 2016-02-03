

const int AIN1_P5 = 5;
const int AIN2_P6 = 6;
const int BIN1_P10 = 10;
const int BIN2_P11 = 11;


void setup() 
{
  pinMode(AIN1_P5, OUTPUT);
  pinMode(AIN2_P6, OUTPUT);
  /*
  pinMode(BIN1_P10, OUTPUT);
  pinMode(BIN2_P11, OUTPUT);
  */
}

void pwmOutputMotorA(unsigned int frequency, float percentDuty)
{
  if(percentDuty > 1) 
  {
    percentDuty = percentDuty / 100.0;
  }

  // 1000ms/ (freq(in herz) / 1000)
  int time = 1000000 / frequency;
  unsigned int delayOn = percentDuty * time;
  unsigned int delayOff = time - percentDuty * time;

  while(true)
  {
    digitalWrite(AIN1_P5, HIGH);
    delayMicroseconds(delayOn);
    digitalWrite(AIN1_P5, LOW);
    delayMicroseconds(delayOff);    
  }
}

void driveMotorA(unsigned int frequency, float percentDuty)
{
  digitalWrite(AIN2_P6, LOW); 
  pwmOutputMotorA(frequency, percentDuty);
}

void loop() 
{
  driveMotorA(1000, 0.95);
}
