#include <Arduino.h>
#include "PS2X_lib.h"  //for v1.6

#define PS2_DAT        50  //14
#define PS2_CMD        51  //15
#define PS2_SEL        53  //16
#define PS2_CLK        52  //17*
#define pressures   true
#define rumble      true

void displayPIDValues();
int readLineSensors() ;
double readLineSensorsAnalog();
void stop_motors()  ;
void motor_control(int MotorDefaultSpeed, int PIDValue);
void setupMotors()  ;
int motor1(int rpm);
int motor2(int rpm);
int motor3(int rpm);
int motor4(int rpm);
int calculate_pid(float error);
void read_pid_values();
void run();
void setupPS2(PS2X);
void north(int speed) ;
void south(int speed) ;
void west(int speed) ;
void east(int speed) ;
void clockwise(int speed);
void counterclockwise(int speed);
void stop_motors();
void PS2ControlLoop();
void readPIDValuesEEPROM()  ;
void writePIDValuesEEPROM() ;
void loopPS2(PS2X);
void mecanumRun(float xSpeed, float ySpeed, float aSpeed);

// void PS2ControlLoop() {
//   ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed
//   if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
//     north(90);
//   }
//   else if(ps2x.Button(PSB_PAD_RIGHT)){
//     east(90);
//   }
//   else if(ps2x.Button(PSB_PAD_LEFT)){
//     west(90);
//   }
//   else if(ps2x.Button(PSB_PAD_DOWN)){
//     south(90);
//   } else {
//     if(!PID_ACTIVE) stop_motors();
//   }
//
//   if(ps2x.ButtonPressed(PSB_START)) {
//     PID_ACTIVE = !PID_ACTIVE;
//     Serial.println("PID Toggled");
//   }           //will be TRUE if button was JUST pressed
// }

int initial_motor = 100;
PS2X ps2x; // create PS2 Controller Class
float Kp = 3, Ki = 1, Kd = 1;

int PID_ACTIVE = 0;
int timeSincePrinted = 0;


void setup() {
  setupMotors();
  stop_motors();
  Serial.begin(9600);

  // Setting up PS2 Controller
  int PS2error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  int type = ps2x.readType();
  if(PS2error == 0 && type == 1){
    Serial.print("Found Controller, configured successful ");
  }

  // Serial.println("Reading Values from EEPROM");

  timeSincePrinted = millis();
}


/*
void loop() {
  float pid = calculate_pid(readLineSensorsAnalog());

  if(PID_ACTIVE) motor_control(initial_motor, pid);

  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed

  if(ps2x.ButtonPressed(PSB_START)){         //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
    PID_ACTIVE = !PID_ACTIVE;
    Serial.println("PID Toggled");
  }
  if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
    Serial.println("north")    ;north(90);
  }
  else if(ps2x.Button(PSB_PAD_RIGHT)){
      east(90);
  }
  else if(ps2x.Button(PSB_PAD_LEFT)){
    west(90);
  }
  else if(ps2x.Button(PSB_PAD_DOWN)){
    south(90);
  } else {

  if(!PID_ACTIVE) {
      stop_motors();
      //Serial.println("stopeed);");
    }
  }

  // These allow dynamic assignment of Kp, Ki and Kd Values
  if(ps2x.ButtonPressed(PSB_TRIANGLE)) Kd += 0.1;
  if(ps2x.ButtonPressed(PSB_CROSS)) Kd -= 0.1
  if(ps2x.ButtonPressed(PSB_L1)) Kp += 0.3;
  if(ps2x.ButtonPressed(PSB_L2)) Kp -= 0.3;
  if(ps2x.ButtonPressed(PSB_R1)) Ki += 0.2;
  if(ps2x.ButtonPressed(PSB_R2)) Ki -= 0.2

  if(ps2x.ButtonPressed(PSB_SQUARE)) {
    Serial.println("The PID Values have been written.");
    writePIDValuesEEPROM();
  }
  if(ps2x.ButtonPressed(PSB_CIRCLE)){
    Serial.println("The PID Values have been Read from EEPROM.");
    readPIDValuesEEPROM();
  }

  if(timeSincePrinted - millis() > 2000) {
    timeSincePrinted = millis();
    //displayPIDValues();
    Serial.print(Kp);
    Serial.print(" :: ");
    Serial.print(Ki);
    Serial.print(" :: ");
    Serial.print(Kd);
    Serial.println(" :: ");
  }
}
*/
//for checking the new motor control

#define C1 39
#define C2 43
#define C3 41
#define C4 37
#define C5 47
#define C6 49
#define C7 48
#define C8 45
int readLineSensorsFront() {
  // ---------- THE INPUT VALUES from the line Sensor {BEGIN}
  int lineError = 0;
  int val1 = digitalRead(C1);
  int val2 = digitalRead(C2);
  int val3 = digitalRead(C3);
  int val4 = digitalRead(C4);
  int val5 = digitalRead(C5);
  int val6 = digitalRead(C6);
  int val7 = digitalRead(C7);
  int val8 = digitalRead(C8);
  // ---------- THE INPUT VALUES from the line Sensor {END}

  // Serial.print("Line Sensors: ");
  // Serial.print(val8); Serial.print(" ");
  // Serial.print(val7); Serial.print(" ");
  // Serial.print(val6); Serial.print(" ");
  // Serial.print(val5); Serial.print(" ");
  // Serial.print(val4); Serial.print(" ");
  // Serial.print(val3); Serial.print(" ");
  // Serial.print(val2); Serial.print(" ");
  // Serial.print(val1); Serial.print(" :: ");

  // applying bit masks
  int val = 0;
  val |= (val8) << 7;
  val |= (val7) << 6;
  val |= (val6) << 5;
  val |= (val5) << 4;
  val |= (val4) << 3;
  val |= (val3) << 2;
  val |= (val2) << 1;
  val |= (val1) << 0;

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
          lineError = 99;
        break;
  }
  // Serial.prinstln("Line Sensor suff");
  return lineError;
}
int readLineSensorsBack() {
  // ---------- THE INPUT VALUES from the line Sensor {BEGIN}
  int lineError = 0;
  int val1 = digitalRead(5);
  int val2 = digitalRead(31);
  int val3 = digitalRead(29);
  int val4 = digitalRead(4);
  int val5 = digitalRead(27);

  // ---------- THE INPUT VALUES from the line Sensor {END}

  Serial.print("Back Line Sensors: ");

  Serial.print(val5); Serial.print(" ");
  Serial.print(val4); Serial.print(" ");
  Serial.print(val3); Serial.print(" ");
  Serial.print(val2); Serial.print(" ");
  Serial.print(val1); Serial.print(" :: ");

  // applying bit masks
  int val = 0;

  val |= (val5) << 4;
  val |= (val4) << 3;
  val |= (val3) << 2;
  val |= (val2) << 1;
  val |= (val1) << 0;

  // here "lineError" is a GLOBAL constant
  switch (val)    {
      case B00001: lineError =  2; break;   // Go right
      case B00010: lineError =  1; break;
      case B00100: lineError =  0; break;
      case B01000: lineError =  -1; break;
      case B10000: lineError = -2; break; // Go left
      default:
          lineError = 99;
        break;
  }
  // Serial.println("Line Sensor suff");
  return lineError;
}


void loop() {
  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed

  int x_axis = 128 - ps2x.Analog(PSS_LX);
  int y_axis = 128 - ps2x.Analog(PSS_LY);
  int rotation = 128 - ps2x.Analog(PSS_RY);

  x_axis = 0;
  y_axis = 50;
  rotation = 0;
  // mecanumRun(x_axis, y_axis,  rotation);

  int error8 = readLineSensorsFront();
  int error5 = readLineSensorsBack();
  // Serial.print(error5);
  // Serial.print(" ");
  // Serial.println(error8);

}
