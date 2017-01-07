#include <Arduino.h>


// Left -> Right
// C8, C7, C6, C5, C4, C3, C2, C1
#define DEBUG_CONSTRAST_SENSORS 0

#define C1 39
#define C2 43
#define C3 41
#define C4 37
#define C5 47
#define C6 49
#define C7 48
#define C8 45

float readLineSensorsFront() {
  float lineError = 0;
  int val1 = digitalRead(C1);
  int val2 = digitalRead(C2);
  int val3 = digitalRead(C3);
  int val4 = digitalRead(C4);
  int val5 = digitalRead(C5);
  int val6 = digitalRead(C6);
  int val7 = digitalRead(C7);


  // applying bit masks
  int val = 0;
  // val |= (val8) << 7;
  val |= (val7) << 6;
  val |= (val6) << 5;
  val |= (val5) << 4;
  val |= (val4) << 3;
  val |= (val3) << 2;
  val |= (val2) << 1;
  val |= (val1) << 0;

  #define PRINT_LINE_SENSOR 0
  if  (PRINT_LINE_SENSOR){
    Serial.print("Front Line Sensors: ");
    Serial.println(val,2);
  }

  switch (val)    {
      case B00000001: lineError =  3; break;
      case B00000011: lineError =  2.5; break;   // Go right
      case B00000111: lineError =  2; break;
      case B00000110: lineError =  1.5; break;
      case B00001110: lineError =  1; break;
      case B00001100: lineError =  0.5; break;
      case B00001000: lineError =  0; break;
      case B00011000: lineError = -0.5; break;
      case B00111000: lineError = -1; break;
      case B00110000: lineError = -1.5; break;
      case B01110000: lineError = -2; break;
      case B01100000: lineError = -2.5; break;
      case B01000000: lineError = -3; break; // Go left
      default:
          lineError = 99;
        break;
  }
   Serial.println(lineError);
  return lineError;
}
