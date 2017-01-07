
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
