#include <Arduino.h>
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.in
 */
int done = 0;
long timeTaken;
double speedOfFrisbee = 0;
long timeOutTime = 0 ;    // Misleading Name, tells the last time LOW detected

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  /*
  Serial.print(digitalRead(2));
  Serial.print(" ");
  Serial.println(digitalRead(3));
  */
  if(timeTaken > 0) {
     Serial.println(speedOfFrisbee);
     timeTaken = 0;
   }
  if(digitalRead(3) == LOW  && (millis() - timeOutTime > 50)) {
    long curTime = millis();
    while(digitalRead(2) != LOW)  ;
    timeTaken = millis() - curTime;
    // timeTaken is in millisecond
    speedOfFrisbee = 0.092 / (timeTaken/1000.0);  // 0.092 is the distance between
                                                // two Sensors
    timeOutTime = millis();
  }
}
