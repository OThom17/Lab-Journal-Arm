// To control the servo with a frequency of 0.2Hz the period ought ot be 1/0.2 = 5 seconds
// 5/360 gives a delay of 0.0139 seconds between servo writes
int Pot = A5;
int ServoA = 10;  // Sevo PWM line
int Average = 0.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
  // Initial Sweep
  
  for (int i = 50; i <= 254; i++){  // 255 is the maximum pwm output of the 
    analogWrite(ServoA, i);
    delay(14);
    Serial.println(i);
  }
  for (int i = 254; i >= 50; i--){  // 180 is the maximum range of the servo motor
    analogWrite(ServoA, i);
    delay(14);
    Serial.println(i);
}
}

void loop() {
  Average = 0;
  for (int i = 0; i < 2; i++){
    Average += analogRead(Pot);
  }
  int Position = floor(Average/2);
  Serial.print("RAW: ");
  Serial.print(Position);
  int x = map(Position, 0, 800, 70, 255); // Scales the value from the potentiometer to the scale of the PWM
  analogWrite(ServoA, x);     // PWM Output
  Serial.print("Scaled: ");
  Serial.println(x);
}


