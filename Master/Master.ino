
#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/JointState.h>

using namespace ros;

NodeHandle  nh;

Servo MountPrimary;
Servo PrimarySecondary;

void cb(const sensor_msgs::JointState&  states){
  int angle1 = (int)(ssttion[0] * 180/3.14);
  int angle2 = (int)(msg.position[1] * 180/3.14);

  MountPrimary.write(angle1);
  PrimarySecondary.write(angle2);
  
  Serial.print("ROS Output Mount: ");
  Serial.println(angle1);
  Serial.print("ROS Output Primary: ");
  Serial.println(angle2);

}


Subscriber<sensor_msgs::JointState>
                        sub("joint_states", cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600);
  Serial.print("Start");
  MountPrimary.attach(9); //attach it to pin 9
  PrimarySecondary.attach(9); //attach it to pin 9
 }

void loop(){
  nh.spinOnce();
  delay(10);
}
