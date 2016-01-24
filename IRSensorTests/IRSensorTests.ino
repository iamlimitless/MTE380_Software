
#include "math.h"

const int IR_SIGNAL_PIN = A0;

float voltageFromDigital(int digitalVal)
{
  //0.005 is 3V / 600 where 600 is the empirically measure ADC output at max. 
  return   0.005 * digitalVal;
}

double irToDistance(int digitalVal)
{
    return 12.361 * pow(0.005 * digitalVal, -1.09); //magic numbers from excel trendline
}


void setup() 
{
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop() 
{
  
  // Give the sensor some time to warm up before reading values (Only really needed for initial startup)
  for(int i = 0; i < 6; i++)
  {
      int sensorVal = analogRead(IR_SIGNAL_PIN);
      delay(1000);
  }

  for(int j = 0; j < 10; j ++)
  {
      int sensorVal = analogRead(IR_SIGNAL_PIN);
      Serial.println(irToDistance(sensorVal));
      delay(1000);
  }
  
  while(true)
  {
    delay(1000);
  }
}
