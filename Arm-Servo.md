# Robot Arm Project - Servo Control

## Controlling a RC servo

```
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
    delay(14);
    Serial.println(i);
  }
  for (int i = 254; i >= 50; i--){  // 180 is the maximum range of the servo motor
    analogWrite(ServoA, i);
    delay(14);
    Serial.println(i);
}
}


```


## Control the servo with a potentiometer

```
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


```

## Robot arm mock up using cardboard 

Using a piece of cardboard make a rudimentary robot arm and affix the servo motor and horn to allow for torque testing.

*** Perfom torque testing 

*** Add torque testing results


Modify the arm to include the second servo

Write a program which uses two potentiometers to control the servos

```
// To control the servo with a frequency of 0.2Hz the period ought ot be 1/0.2 = 5 seconds
// 5/360 gives a delay of 0.0139 seconds between se  rvo writes

// Pin declaration for potentiometers
int PotA = A5;
int PotB = A6;

// Servo Declaration
int ServoA = 10;  // Sevo A PWM line
int ServoB = 11;  // Sevo B PWM line

// Averaging Variables to reduce noise
int AverageA = 0;
int AverageB = 0;

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
// Reset the Averages
  int AverageA = 0;
  int AverageB = 0;
  
// Fill the variable with three samples - Reduces the noise
  for (int i = 0; i <= 2; i++){
    AverageA += analogRead(PotA);
  }
  int PositionA = floor(AverageA/3);


// Fill the variable with three samples - Reduces the noise
  for (int i = 0; i <= 2; i++){
    AverageB += analogRead(PotB);
  }
  int PositionB = floor(AverageB/3);
  
  
 // Debugging Code 
  Serial.print("RAW A: ");
  Serial.print(PositionA);
  
  Serial.print("RAW B: ");
  Serial.print(PositionB);
  
 // Translate the analogue value to the PWM range
  int xA = map(PositionA, 0, 800, 70, 255); // Scales the value from the potentiometer to the scale of the PWM
  int xB = map(PositionB, 0, 800, 70, 255); // Scales the value from the potentiometer to the scale of the PWM

 // Write each value to the respective servo 
  analogWrite(ServoA, xA);     // PWM Output
  analogWrite(ServoB, xB);     // PWM Output

  Serial.print("Scaled A: ");
  Serial.println(xA);
  Serial.print("Scaled B: ");
  Serial.println(xB);
}


```



















