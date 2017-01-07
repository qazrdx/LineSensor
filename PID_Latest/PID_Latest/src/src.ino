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

  Serial.println("Reading Values from EEPROM");
  //readPIDValuesEEPROM();
  //setupPS2(ps2x);
  timeSincePrinted = millis();
}
void loopPS2(PS2X);
void loop() {
  float pid = calculate_pid(readLineSensorsAnalog());
  //Serial.print(pid);
//  Serial.print("  ");
  if(PID_ACTIVE) motor_control(initial_motor, pid);
  //  run();
  //PS2ControlLoop();
  //loopPS2(ps2x);

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
/*
   if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if(ps2x.Button(PSB_L3))
      Serial.println("L3 pressed");
    if(ps2x.Button(PSB_R3 ))
      Serial.println("R3 pressed");
    if(ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
    if(ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
    if(ps2x.Button(PSB_TRIANGLE))
      Serial.println("Triangle pressed");
  }

  if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    Serial.println("Square just released");

  if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }
  */

  // These allow dynamic assignment of Kp, Ki and Kd Values
  if(ps2x.ButtonPressed(PSB_TRIANGLE)) Kd += 0.1;
  if(ps2x.ButtonPressed(PSB_CROSS)) Kd -= 0.1;

  if(ps2x.ButtonPressed(PSB_SQUARE))
  {
    Serial.println("The PID Values have been written.");
    writePIDValuesEEPROM();
  }

  if(ps2x.ButtonPressed(PSB_CIRCLE)){
    Serial.println("The PID Values have been Read from EEPROM.");
    readPIDValuesEEPROM();
  }

  if(ps2x.ButtonPressed(PSB_L1)) Kp += 0.3;
  if(ps2x.ButtonPressed(PSB_L2)) Kp -= 0.3;
  if(ps2x.ButtonPressed(PSB_R1)) Ki += 0.2;
  if(ps2x.ButtonPressed(PSB_R2)) Ki -= 0.2;
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

void PS2ControlLoop() {
  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed
  if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
    north(90);
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
    if(!PID_ACTIVE) stop_motors();
  }

  if(ps2x.ButtonPressed(PSB_START)) {
    PID_ACTIVE = !PID_ACTIVE;
    Serial.println("PID Toggled");
  }           //will be TRUE if button was JUST pressed
}
