#include <Arduino.h>
// for saving the values of Kp, Ki, Kd to EEPROM
#include <EEPROM.h>


#define DEBUG_PID_VALUES 1
#define TIME_INTERVAL_FOR_PID_CALCULATION 20

#define POT_1 A10
#define POT_2 A9
#define POT_3 A8

extern float Kp, Ki, Kd ;
int previousError = 0, timeSinceLastCalculated = 0;

int calculate_pid(int error) {
  if(millis() - timeSinceLastCalculated > TIME_INTERVAL_FOR_PID_CALCULATION)  {
    int P = error;
    int I = I + error;
    int D = error - previousError;

    previousError = error;
    timeSinceLastCalculated = millis();
    return (Kp * P) + (Ki * I) + (Kd * D);
  }

  // to initialize timeSinceLastCalculated
  if(timeSinceLastCalculated == 0) timeSinceLastCalculated = millis();
  return 0;
}

void displayPIDValues()  {
  /*
  Kp = constrain(map(analogRead(A10), 0, 1024, 1024, 0) / 100.0, 0 , 10);
  Ki = constrain(map(analogRead(A9), 0, 1024, 1024, 0) / 700.0, 0 , 3);
  Kd = constrain(analogRead(A8) / 100.0, 0 , 10);
*/
  #ifdef DEBUG_PID_VALUES
    Serial.print(" P,I and D Constants:");
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(Ki);
    Serial.print(" ");
    Serial.print(Kd);
    Serial.print(" ");
    Serial.println();
  #endif
}

void readPIDValuesEEPROM()  {
  //      BE CAREFUL WHILE CALLING THIS. It will overwrite the current values.

  // An eeprom value can be from 0-255.
  // therefore all the variables involved are manipulated before inserting.
  // Hence, they are converted back while reading.

  // This Scheme gives a range of 0.0 - 25.5
  Kp = EEPROM.read(0);        // Kp is at 0th address
  Ki = EEPROM.read(1);        // Ki is at 1st address
  Kd = EEPROM.read(2);        // Kd is at 2nd address

  Kp = Kp / 10;
  Ki = Ki / 10;
  Kd = Ki / 10;
}

void writePIDValuesEEPROM() {
  // Careful with this one too...
  EEPROM.write((int)(Kp*10), 0);    // converting and casting to int
  EEPROM.write((int)(Ki*10), 1);    // converting and casting to int
  EEPROM.write((int)(Kd*10), 2);    // converting and casting to int
}
