#include <Arduino.h>


// this is for clockwise, each motor
#define cw_1 HIGH
#define cw_2 HIGH
#define cw_3 LOW
#define cw_4 LOW

// for anti-clockwise, automactically defined on basis of clockwise values
#define acw_1 (!cw_1)
#define acw_2 (!cw_2)
#define acw_3 (!cw_3)
#define acw_4 (!cw_4)

// for varying PWM seperately in each motor.    change the below constant at attain desired Result
int calibrated_pwm1(int x) {return (x*.75);}  // RB 0.85
int calibrated_pwm2(int x) {return (x*1.3);}   // lB 1.1
int calibrated_pwm3(int x) {return (x*1.1);}     // RF 1.1
int calibrated_pwm4(int x) {return (x*1.0);}   // LF 1.1


int mot1 = 17; int pwm1 = 10;
int mot2 = 14; int pwm2 = 13;
int mot3 = 16; int pwm3 = 11;
int mot4 = 15; int pwm4 = 12;

void setupMotors()  {
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(mot3, OUTPUT);
  pinMode(mot4, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
}

void stop_motors()  {
  analogWrite(pwm1, 0);  //Right
  analogWrite(pwm2, 0); //Left Motor Speed
  analogWrite(pwm3, 0);  //Left Motor Speed
  analogWrite(pwm4, 0);  //Right
}

void motor_control(int MotorDefaultSpeed, int PIDValue)
{
  // Calculating the effective motor speed:
  int left_motor_speed = MotorDefaultSpeed + PIDValue;
  int right_motor_speed = MotorDefaultSpeed - PIDValue;


  analogWrite(pwm1, calibrated_pwm1(right_motor_speed));  //Right
  analogWrite(pwm2, calibrated_pwm2(left_motor_speed));
  analogWrite(pwm3, calibrated_pwm3(right_motor_speed));
  analogWrite(pwm4, calibrated_pwm4(left_motor_speed));  //Right

  digitalWrite(mot1, acw_1);
  digitalWrite(mot2, acw_2);
  digitalWrite(mot3, acw_3);
  digitalWrite(mot4, acw_4);
}

void go_forward_PID(int speed, double variance)  {
  int v = (int) variance;
  digitalWrite(mot1, cw_1);
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, cw_3);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(speed - v));    // --> 3      h -> clo
  analogWrite(pwm2, calibrated_pwm1(speed + v));    // --> 2      h -> anti
  analogWrite(pwm3, calibrated_pwm1(speed + v));    // --> 1      h -> anti
  analogWrite(pwm4, calibrated_pwm1(speed - v));    // --> 4      h -> clock
}

void south(int speed) {
  digitalWrite(mot1, cw_1);
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, cw_3);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void north(int speed) {
  digitalWrite(mot1, acw_1);
  digitalWrite(mot2, acw_2);
  digitalWrite(mot3, acw_3);
  digitalWrite(mot4, acw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void east(int speed) {
  digitalWrite(mot1, acw_1);
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, cw_3);
  digitalWrite(mot4, acw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void west(int speed) {
  digitalWrite(mot1, cw_1);
  digitalWrite(mot2, acw_2);
  digitalWrite(mot3, acw_3);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void northeast(int speed) {
  digitalWrite(mot1, cw_1);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void northwest(int speed) {
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, cw_3);

  analogWrite(pwm1, 0);
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, 0);
}
void southeast(int speed) {
  digitalWrite(mot2, acw_2);
  digitalWrite(mot3, acw_3);

  analogWrite(pwm1, 0);
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, 0);
}
void southwest(int speed) {
  digitalWrite(mot1, acw_1);
  digitalWrite(mot4, acw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void stop_moving() {
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);
}
void clockwise(int speed) {
  digitalWrite(mot1, cw_1);
  digitalWrite(mot2, acw_2);
  digitalWrite(mot3, cw_3);
  digitalWrite(mot4, acw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void counterclockwise(int speed) {
  digitalWrite(mot1, acw_1);
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, acw_3);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(speed));
  analogWrite(pwm2, calibrated_pwm2(speed));
  analogWrite(pwm3, calibrated_pwm3(speed));
  analogWrite(pwm4, calibrated_pwm4(speed));
}
void run()  {
  digitalWrite(mot1, cw_1);
  digitalWrite(mot2, cw_2);
  digitalWrite(mot3, cw_3);
  digitalWrite(mot4, cw_4);

  analogWrite(pwm1, calibrated_pwm1(70));
  analogWrite(pwm2, calibrated_pwm2(70));
  analogWrite(pwm3, calibrated_pwm3(70));
  analogWrite(pwm4, calibrated_pwm4(70));
}
