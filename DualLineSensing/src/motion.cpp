#include <Arduino.h>

int mot4 = 17; int pwm4 = 10;
int mot3 = 14; int pwm3 = 13;
int mot2 = 16; int pwm2 = 11;
int mot1 = 15; int pwm1 = 12;
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
int calibrated_pwm4(int x) {return (x*.85);}  // RB 0.85
int calibrated_pwm3(int x) {return (x*1.3);}   // lB 1.1
int calibrated_pwm2(int x) {return (x*1.1);}     // RF 1.1
int calibrated_pwm1(int x) {return (x*1.0);}   // LF 1.1


void setMotor1(float speed) {
  if(speed > 0) {digitalWrite(mot1, cw_1);  analogWrite(pwm1,  calibrated_pwm1(speed));}
  else          {digitalWrite(mot1, acw_1); analogWrite(pwm1, calibrated_pwm1(-speed));}
}

void setMotor2(float speed) {
  if(speed > 0) {digitalWrite(mot2, cw_2);  analogWrite(pwm2,  calibrated_pwm1(speed));}
  else          {digitalWrite(mot2, acw_2); analogWrite(pwm2, calibrated_pwm1(-speed));}
}

void setMotor3(float speed) {
  if(speed > 0) {digitalWrite(mot3, cw_3);  analogWrite(pwm3,  calibrated_pwm3(speed));}
  else          {digitalWrite(mot3, acw_3); analogWrite(pwm3, calibrated_pwm3(-speed));}
}

void setMotor4(float speed) {
  if(speed > 0) {digitalWrite(mot4, cw_4);  analogWrite(pwm4,  calibrated_pwm4(speed));}
  else          {digitalWrite(mot4, acw_4); analogWrite(pwm4, calibrated_pwm4(-speed));}
}

void setEachMotorSpeed(float speed1, float speed2, float speed3, float speed4)
{
  setMotor1(speed1);
  setMotor2(speed2);
  setMotor3(speed3);
  setMotor4(speed4);
  Serial.println(speed2);

}




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

void mecanumRun(float xSpeed, float ySpeed, float aSpeed) {
  int maxLinearSpeed = 100;
  // float speed1 = ySpeed - xSpeed + aSpeed;
  // float speed2 = ySpeed + xSpeed - aSpeed;
  // float speed3 = ySpeed - xSpeed - aSpeed;
  // float speed4 = ySpeed + xSpeed + aSpeed;

  float speed1 = ySpeed - aSpeed + xSpeed;
  float speed2 = ySpeed + aSpeed - xSpeed;
  float speed3 = ySpeed - aSpeed - xSpeed;
  float speed4 = ySpeed + aSpeed + xSpeed;

  float max = speed1;
  if (max < speed2)   max = speed2;
  if (max < speed3)   max = speed3;
  if (max < speed4)   max = speed4;

  if (max > maxLinearSpeed)
  {
    speed1 = speed1 / max * maxLinearSpeed;
    speed2 = speed2 / max * maxLinearSpeed;
    speed3 = speed3 / max * maxLinearSpeed;
    speed4 = speed4 / max * maxLinearSpeed;
  }

  setEachMotorSpeed(speed1, speed2, speed3, speed4);
}

//
// void motor_control(int MotorDefaultSpeed, int PIDValue)
// {
//   // Calculating the effective motor speed:
//   int left_motor_speed = MotorDefaultSpeed + PIDValue;
//   int right_motor_speed = MotorDefaultSpeed - PIDValue;
//
//
//   analogWrite(pwm1, calibrated_pwm1(right_motor_speed));  //Right
//   analogWrite(pwm2, calibrated_pwm2(left_motor_speed));
//   analogWrite(pwm3, calibrated_pwm3(right_motor_speed));
//   analogWrite(pwm4, calibrated_pwm4(left_motor_speed));  //Right
//
//   digitalWrite(mot1, acw_1);
//   digitalWrite(mot2, acw_2);
//   digitalWrite(mot3, acw_3);
//   digitalWrite(mot4, acw_4);
// }
//
// void go_forward_PID(int speed, double variance)  {
//   int v = (int) variance;
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, cw_3);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed - v));    // --> 3      h -> clo
//   analogWrite(pwm2, calibrated_pwm1(speed + v));    // --> 2      h -> anti
//   analogWrite(pwm3, calibrated_pwm1(speed + v));    // --> 1      h -> anti
//   analogWrite(pwm4, calibrated_pwm1(speed - v));    // --> 4      h -> clock
// }
//
// void south(int speed) {
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, cw_3);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void north(int speed) {
//   digitalWrite(mot1, acw_1);
//   digitalWrite(mot2, acw_2);
//   digitalWrite(mot3, acw_3);
//   digitalWrite(mot4, acw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void east(int speed) {
//   digitalWrite(mot1, acw_1);
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, cw_3);
//   digitalWrite(mot4, acw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void west(int speed) {
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot2, acw_2);
//   digitalWrite(mot3, acw_3);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void northeast(int speed) {
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, 0);
//   analogWrite(pwm3, 0);
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void northwest(int speed) {
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, cw_3);
//
//   analogWrite(pwm1, 0);
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, 0);
// }
// void southeast(int speed) {
//   digitalWrite(mot2, acw_2);
//   digitalWrite(mot3, acw_3);
//
//   analogWrite(pwm1, 0);
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, 0);
// }
// void southwest(int speed) {
//   digitalWrite(mot1, acw_1);
//   digitalWrite(mot4, acw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, 0);
//   analogWrite(pwm3, 0);
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
void stop_moving() {
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);
}
// void clockwise(int speed) {
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot2, acw_2);
//   digitalWrite(mot3, cw_3);
//   digitalWrite(mot4, acw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void counterclockwise(int speed) {
//   digitalWrite(mot1, acw_1);
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, acw_3);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(speed));
//   analogWrite(pwm2, calibrated_pwm2(speed));
//   analogWrite(pwm3, calibrated_pwm3(speed));
//   analogWrite(pwm4, calibrated_pwm4(speed));
// }
// void run()  {
//   digitalWrite(mot1, cw_1);
//   digitalWrite(mot2, cw_2);
//   digitalWrite(mot3, cw_3);
//   digitalWrite(mot4, cw_4);
//
//   analogWrite(pwm1, calibrated_pwm1(70));
//   analogWrite(pwm2, calibrated_pwm2(70));
//   analogWrite(pwm3, calibrated_pwm3(70));
//   analogWrite(pwm4, calibrated_pwm4(70));
// }
