#include <Servo.h>

const int MOTOR_PIN = 2;
const int STOP_BTN_PIN = 4;
const int MOTOR_OFF = 1000;
const int MOTOR_FULL = 2000;

volatile bool btnPressed = false;
Servo propMotor;

//interrupt handler for btn
void handleBtnPressed()
{
  detachInterrupt(digitalPinToInterrupt(STOP_BTN_PIN));
  btnPressed = true;
}

void setup() {
  pinMode(STOP_BTN_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  
  propMotor.attach(MOTOR_PIN);

  attachInterrupt(digitalPinToInterrupt(STOP_BTN_PIN), handleBtnPressed, FALLING);
}

void setThrottle(int percentOfMax)
{
  if(percentOfMax > 100)
  {
    return;
  }
  propMotor.writeMicroseconds(1000 + (1000 * (percentOfMax / 100.0)));
}

void loop() {
  propMotor.writeMicroseconds(MOTOR_OFF); //Arm ESC

  for(int percent = 0; percent <= 100; percent += 10)
  {    
    propMotor.writeMicroseconds(MOTOR_OFF);
    while(!btnPressed); //power consumption issue? maybe delay so not so busy looping
    delay(20); //debounce
    setThrottle(percent);
    while(!btnPressed);
    delay(20); //debounce
  }
  

}
