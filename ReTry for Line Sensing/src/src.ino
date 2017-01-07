#include <Arduino.h>
#include "PS2X_lib.h"  //for v1.6
#include "NewPing.h"

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
int calculate_pid(float error);
void read_pid_values();
void setupPS2(PS2X);
void stop_motors();
void PS2ControlLoop();
void readPIDValuesEEPROM()  ;
void writePIDValuesEEPROM() ;
void loopPS2(PS2X);
void mecanumRun(float xSpeed, float ySpeed, float aSpeed);
float readLineSensorsFront();

int initial_motor = 100;
PS2X ps2x; // create PS2 Controller Class
float Kp = 3, Ki = 1, Kd = 1;

int PID_ACTIVE = 0;
int timeSincePrinted = 0;

float ultrasonicDistance1 = 0, ultrasonicDistance2 = 0,ultrasonicCheckTime = 0;
float    fixedDistance=9;


#define TRIGGER_PIN_1  32  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define TRIGGER_PIN_2  26  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_1     30  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN_2     28  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE  200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing US1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing US2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

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

  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(ECHO_PIN_2, INPUT);

  ultrasonicCheckTime = millis();

}


void updateUltrasonic() {
  if(millis() - ultrasonicCheckTime > 200) {
    ultrasonicDistance1 = US1.ping_cm();
    ultrasonicDistance2 = US2.ping_cm();
    Serial.println(ultrasonicDistance1);
    ultrasonicCheckTime = millis();
  }
}

float getDistance() {
  return (ultrasonicDistance1 < ultrasonicDistance2 ) ? ultrasonicDistance1 : ultrasonicDistance2;
}

float getAngle()  {
  if(ultrasonicDistance1 > ultrasonicDistance2)
  {
    double param1 = (ultrasonicDistance1-ultrasonicDistance2)/fixedDistance;
    double angle1= acos (param1)* (180/3.142);
    Serial.print("left");
    Serial.print("\t");
    Serial.print("angle1: ");
    Serial.print(angle1);
    return (90.0 - angle1);
  }
  else if (ultrasonicDistance1 < ultrasonicDistance2)
  {
    double param2 = (ultrasonicDistance1-ultrasonicDistance2)/fixedDistance;
    double angle2= acos (param2)* (180/3.142);
    Serial.print("right");
    Serial.print("\t");
    Serial.print("angle2: ");
    Serial.print(angle2);
    return -(90.0 - angle2);
  }
}

void loop() {
  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed

  //int error7 = readLineSensorsFront();

  updateUltrasonic();
  float distance = getDistance();
  //float angle = getAngle();



  #define QUADMOTORS_RUNNING 0
  if(QUADMOTORS_RUNNING){
    int x_axis = 0;
    int y_axis = 50;
    int rotation = 0;
    mecanumRun(x_axis, y_axis,  rotation);
  }
}
