
// To control the servo with a frequency of 0.2Hz the period ought ot be 1/0.2 = 5 seconds
// 5/360 gives a delay of 0.0139 seconds between servo writes

int ServoA = 10;


void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
  pinMode(ServoA, OUTPUT); 
}

void loop() {
// Testing the bounds of the 
  analogWrite(ServoA, 50);
  delay(200);

  analogWrite(ServoA, 180);
  delay(200);

  analogWrite(ServoA, 254);
  delay(200);

  analogWrite(ServoA, 180);
  delay(200);

}



