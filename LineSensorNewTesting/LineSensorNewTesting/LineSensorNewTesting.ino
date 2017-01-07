//#include "readSensorValues.cpp"

int readLineSensors();

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

int arr[8] = { 39, 43, 41, 37,  47, 49, 48, 45};

void test() {
  int i;
  for(i = 0; i < 8; i++)  {
    Serial.print(digitalRead(arr[i]));
    Serial.print(" ");
  }
    Serial.println();
}

// the loop function runs over and over again forever
void loop() {
  //test();
  //readLineSensors();
  if(0) {
  double k = analogRead(A8);
  k = k / 128;
  k = k-3;
  if(k > 4 || k < -4) { // out of area
  }
  Serial.println(k);
}
 test();
 //Serial.println(digitalRead(48));

  delay(100);
}
