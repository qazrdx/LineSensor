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



// for reading the sensors and getting the error values
int readLineSensors() {
  // ---------- THE INPUT VALUES from the line Sensor {BEGIN}
  int lineError = 0;

  #ifdef DEBUG_CONSTRAST_SENSORS
    int val1 = digitalRead(C1);
    int val2 = digitalRead(C2);
    int val3 = digitalRead(C3);
    int val4 = digitalRead(C4);
    int val5 = digitalRead(C5);
    int val6 = digitalRead(C6);
    int val7 = digitalRead(C7);
    int val8 = digitalRead(C8);
    // ---------- THE INPUT VALUES from the line Sensor {END}

    Serial.print("Line Sensors: ");
    Serial.print(val8); Serial.print(" ");
    Serial.print(val7); Serial.print(" ");
    Serial.print(val6); Serial.print(" ");
    Serial.print(val5); Serial.print(" ");
    Serial.print(val4); Serial.print(" ");
    Serial.print(val3); Serial.print(" ");
    Serial.print(val2); Serial.print(" ");
    Serial.print(val1); Serial.print(" :: ");
  #endif

  // applying bit masks
  int val = 0;
  val |= digitalRead(val8) << 7;
  val |= digitalRead(val7) << 6;
  val |= digitalRead(val6) << 5;
  val |= digitalRead(val5) << 4;
  val |= digitalRead(val4) << 3;
  val |= digitalRead(val3) << 2;
  val |= digitalRead(val2) << 1;
  val |= digitalRead(val1) << 0;

  // here "lineError" is a GLOBAL constant
  switch (val)    {
      case B00000001: lineError =  4; break;   // Go right
      case B00000010: lineError =  3; break;
      case B00000100: lineError =  2; break;
      case B00001000: lineError =  1; break;
      case B00010000: lineError = -1; break;
      case B00100000: lineError = -2; break;
      case B01000000: lineError = -3; break;
      case B10000000: lineError = -4; break;   // Go left
      default:
        #ifdef DEBUG_CONSTRAST_SENSORS
          Serial.print("Line Sensors Not giving a valid Response, ");
          Serial.println(val);
        #endif
        break;
  }
  return lineError;
}


// Legacy Function
#if 0
void read_sensor_values()
{

  sensor[0] = digitalRead(41);
  sensor[1] = digitalRead(43);
  sensor[2] = digitalRead(45);
  sensor[3] = digitalRead(37);
  sensor[4] = digitalRead(47);


  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //&&(sensor[5]==0)) //110000
    curr_pos = LEFT;
    error = 4;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //&&(sensor[5]==0)) //111000
    curr_pos = LEFT;
    error = 3;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) { //&&(sensor[5]==0)) //011000
    curr_pos = LEFT;
    error = 1.5;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) { //&&(sensor[5]==0)) //011100
    curr_pos = LEFT;
    error = 1;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) { //&&(sensor[5]==0)) //001100
    curr_pos = MIDDLE;
    error = 0;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) { //&(sensor[5]==0))
    curr_pos = RIGHT;
    error = -1;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)) { //&&(sensor[5]==0))
    curr_pos = RIGHT;
    error = -1.5;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) { //&&(sensor[5]==1))
    curr_pos = RIGHT;
    error = -3;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) { //&&(sensor[5]==1))
    curr_pos = RIGHT;
    error = -4;
  }

  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) //&&(sensor[5]==0))
  {
    if (curr_pos == MIDDLE) {
      curr_pos = MIDDLE;
      error = 0;
    }
    if (curr_pos == LEFT) {
      curr_pos = LEFT;
      error = 5;
    }
    if (curr_pos == RIGHT) {
      curr_pos = RIGHT;
      error = -5;
    }
  }
  //  Serial.print(curr_pos);
  //  Serial.print(" ");
  //  Serial.println(error);
  //  //if (prev != 0)  prev = error;
  // Serial.println(error);
}
#endif
