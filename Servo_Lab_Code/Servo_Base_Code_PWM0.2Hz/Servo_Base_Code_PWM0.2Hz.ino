// To control the servo with a frequency of 0.2Hz the period ought ot be 1/0.2 = 5 seconds
// 5/360 gives a delay of 0.0139 seconds between servo writes
int Pot = A0;
int ServoA = 10;  // Sevo PWM line

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
}

void loop() {
  // Sweep
  for (int i = 50; i <= 254; i++){  // 255 is the maximum pwm output of the 
    analogWrite(ServoA, i);
    delay(12.5);                    // Time delay to achieve the 0.2Hz and 5 second frequency
    Serial.println(i);
  }
  for (int i = 254; i >= 50; i--){  // 180 is the maximum range of the servo motor
    analogWrite(ServoA, i);
    delay(12.5);
    Serial.println(i);
  }

}


