#include <NewPing.h>

const int ULTRA_SONIC_PIN = 47;

NewPing sonar(ULTRA_SONIC_PIN, ULTRA_SONIC_PIN, 300);

void setup() 
{
  Serial.begin(9600);
}


void loop() 
{
	delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
 	unsigned long distance = sonar.ping_cm();
 	Serial.printf("Distance in cm is %s\n", distance);
}
