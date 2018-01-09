#include <Servo.h>
Servo ServoA;
// To control the servo with a frequency of 0.2Hz the period ought ot be 1/0.2 = 5 seconds
// 5/360 gives a delay of 0.0139 seconds between servo writes
int Pot = A0;


void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
  ServoA.attach(10);
}

void loop() {
  for (int i = 0; i <= 180; i++){  // 180 is the maximum range of the servo motor
    ServoA.write(i);
    delay(14);
  }
  for (int i = 180; i >= 0; i--){  // 180 is the maximum range of the servo motor
    ServoA.write(i);
    delay(14);
  }  
}



