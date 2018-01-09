#include <TimerOne.h>
#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/JointState.h>

//Macro Declarations
#define microsteps 50

using namespace ros;

NodeHandle  nh;

// Servo object initialsation
Servo MountPrimary;
Servo PrimarySecondary;

// Function Prototype
void Write_Stepper(int);  
void timerISR(void);

// Global Declarations
int Stepper_Scaler;  // Determines the rotational speed and direction of the stepper motor
int pulseDelay = 1;  // Used to scale the motor's speed 1 by default
int a[microsteps];   // Initialising the LUT arrays
int b[microsteps];
int idxG = 0;        // Global indexer to track sine/cosine wave progression and resetting


// Call back function
void cb(const sensor_msgs::JointState&  states){
  int MountPrimary_Angle     = (int)(states.position[1] * 180/3.14);  // Map to 360 degrees
  int PrimarySecondary_Angle = (int)(states.position[2] * 180/3.14);  // Map to 360 degrees
  int Stepper_Scaler         = (int)(states.position[0]);              // Take as a raw value
    
// Write values to the motors
  MountPrimary.write(MountPrimary_Angle);
  PrimarySecondary.write(PrimarySecondary_Angle); 
 
  if (Stepper_Scaler = 1.57){  // If the slider is in the central position    
    Timer1.restart();           //Reset the timer to 0
    Timer1.stop();             //Stop the timer and therefore the stepper motor
  }
  //If the slider is furtheraway the speed should increase and vice versa - pulseDelay is varied
  else {  
     float Scaler = (abs(Stepper_Scaler) / 1.57);  // 0 - 1.57 scaled to 0 - 1
     pulseDelay *= Scaler;
  }
 
 
 
  
}

//ROS communication - JountState topic
Subscriber<sensor_msgs::JointState> sub("joint_states", cb);


void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  // Serial initialisation
  Serial.begin(9600);

  // Define the pins for the servo objects
  MountPrimary.attach(9); //attach it to pin 9
  PrimarySecondary.attach(8); //attach it to pin 9
  
  
  // Create the sine wave LUT's
for (int idx = 0; idx < microsteps; idx++){        // 100 seperate steps per physical step of the stepper motor
  a[idx] = 255 * (sin((2* PI* idx)/ microsteps));// Following the Amplitude * Sin((ω/Step) * t) formula
  b[idx] = 255 * (cos((2* PI* idx)/ microsteps));// As above
  
  /* Print the sine and cosine look up tables through the serial port
  Serial.print("A: ");
  Serial.print(a[idx]);
  Serial.print("B: ");
  Serial.println(b[idx]); 
  */
  
 }

  //Stepper motor pin initilisation
  pinMode(12, OUTPUT);  // Channel A - Polarity (H = Forwards, L = Backwards)
  pinMode(13, OUTPUT);  // Channel B - Polarity (H = Forwards, L = Backwards)
  pinMode(9, OUTPUT);   //brake A
  pinMode(8, OUTPUT);   //brake B

  digitalWrite(9, LOW);  //Let both brake A and B off
  digitalWrite(8, LOW);
  
  //Timer 1 Initialisation
  Timer1.initialize(10); // set a timer of n microseconds
  Timer1.attachInterrupt( timerISR ); // attach the service routine here  
}




void loop(){
    nh.spinOnce();
    delay(10); 
}


void WriteMS(int Value, int ChanAnalog, int ChanDigit){  // Responsible for changing the direction of the motor
  int AbsVal = abs(Value);        // Reduces the value to a positive integer value
  analogWrite(ChanAnalog, AbsVal);// Write that value to the stepper motor
  if (Value > 0){                 //Determine which poles to enable based in the sign of the raw value
    digitalWrite(ChanDigit, HIGH);
  }
  else{
    digitalWrite(ChanDigit, LOW);
  }
}



void timerISR(){
  // Need to change direction of the motor - is Stepper_Scaler is negative one direction and vice versa
  
    WriteMS(a[idxG], 3, 12);
    WriteMS(b[idxG], 11, 13);
    
    idxG++;
    
    if (idxG >= microsteps){  // Reset the step once complete
      idxG = 0;
    }
    delayMicroseconds(pulseDelay);
}
