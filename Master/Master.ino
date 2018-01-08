#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/JointState.h>

using namespace ros;

NodeHandle  nh;

// Servo object initialsation
Servo MountPrimary;
Servo PrimarySecondary;

// Call back function
void cb(const sensor_msgs::JointState&  states){
  int MountPrimary_Angle     = (int)(states.position[1] * 180/3.14);  // Map to 360 degrees
  int PrimarySecondary_Angle = (int)(states.position[2] * 180/3.14);  // Map to 360 degrees
  int Stepper_Angle          = (int)(states.position[0];              // Take as a raw value
  
  
  MountPrimary.write(MountPrimary_Angle);
  PrimarySecondary.write(PrimarySecondary_Angle);
}


Subscriber<sensor_msgs::JointState> sub("joint_states", cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  // Serial initialisation
  Serial.begin(9600);

  // Define the pins for the servo objects
  MountPrimary.attach(9); //attach it to pin 9
  PrimarySecondary.attach(8); //attach it to pin 9
  
  
  // Create the sine wave LUT
for (int idx = 0; idx < microsteps; idx++){        // 100 seperate steps per physical step of the stepper motor
  a[idx] = 255 * (sin((2* PI* idx)/ microsteps));// Following the Amplitude * Sin((ω/Step) * t) formula
  b[idx] = 255 * (cos((2* PI* idx)/ microsteps));// As above
  
  // Print the sine and cosine look up tables through the serial port
  Serial.print("A: ");
  Serial.print(a[idx]);
  Serial.print("B: ");
  Serial.println(b[idx]); 
  
 }

  pinMode(12, OUTPUT);  // Channel A - Polarity (H = Forwards, L = Backwards)
  pinMode(13, OUTPUT);  // Channel B - Polarity (H = Forwards, L = Backwards)
  pinMode(9, OUTPUT);   //brake A
  pinMode(8, OUTPUT);   //brake B

  digitalWrite(9, LOW);  //Let both brake A and B off
  digitalWrite(8, LOW);
}


void loop(){
  nh.spinOnce();
  delay(10);
}




void Stepper_Control(){
    WriteMS(a[idxG], 3, 12);
    WriteMS(b[idxG], 11, 13);

    Serial.print(a[idxG]);
    Serial.print(":  ");
    Serial.println(b[idxG]);

    
    idxG++;
    if (idxG >= microsteps){  // Reset the step once complete
      idxG = 0;
    }
    delayMicroseconds(pulseDelay);  
}


void WriteMS(int Value, int ChanAnalog, int ChanDigit){
  int AbsVal = abs(Value);        // Reduces the value to a positive integer value
  analogWrite(ChanAnalog, AbsVal);// Write that value to the stepper motor
  if (Value > 0){                 //Determine which poles to enable based in the sign of the raw value
    digitalWrite(ChanDigit, HIGH);
  }
  else{
    digitalWrite(ChanDigit, LOW);
  }
}


