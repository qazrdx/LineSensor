#define dir 7
#define pwm 5
int pot;
//int mapped;
boolean switch1, switch2;

void setup() {

  Serial.begin(9600);
  //pinMode(A1,INPUT);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  pinMode(2, INPUT);
  pinMode(4, INPUT);

  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);

}

void loop() {

  //pot = analogRead(A1);
  //mapped = map(pot,0,1023,0,255);
  switch1 = digitalRead(2);
    switch2 = digitalRead(4);

  if (switch1 == true && switch2 == false) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, 100);
    switch1 == false;
    Serial.println("UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP");
    delay(30);
  }

  else if (switch1 == false && switch2 == true) {
    digitalWrite(dir, LOW);
    analogWrite(pwm, 100);
    switch2 = false;
    Serial.println("DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN");
    delay(30);

  }

  else {
    analogWrite(pwm, 0);
  }


  //Serial.println("Speed");
  //Serial.println(mapped);
}
